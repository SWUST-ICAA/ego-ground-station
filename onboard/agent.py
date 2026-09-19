#!/usr/bin/env python3
"""SSH-managed ROS1 mission agent. Unix socket only; no network service."""
import argparse
import collections
import fcntl
import json
import math
import os
import signal
import socket
import socketserver
import subprocess
import threading
import time
from geo import to_local
from mission import Mission

SOCKET = '/tmp/fast-drone-ground-station.sock'
VERSION = '1.2.0'


def request(payload):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(20)
        client.connect(SOCKET)
        client.sendall(json.dumps(payload, allow_nan=False).encode()+b'\n')
        data = b''
        while not data.endswith(b'\n') and len(data) < 262144:
            chunk = client.recv(16384)
            if not chunk:
                break
            data += chunk
        return json.loads(data)


class Agent:
    def __init__(self, aircraft):
        import rospy
        from mavros_msgs.msg import State, ExtendedState, PositionTarget
        from sensor_msgs.msg import NavSatFix, BatteryState, PointCloud2
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String, Float64
        from geometry_msgs.msg import PoseStamped
        from traj_utils.msg import Bspline
        from quadrotor_msgs.msg import PositionCommand
        self.rospy = rospy
        self.aircraft = aircraft
        self.session_id=str(time.time_ns())
        self.lock = threading.RLock()
        self.data_lock = threading.RLock()
        self.data = {}
        self.odom_buffer = collections.deque(maxlen=100)
        self.samples = collections.deque(maxlen=10)
        self.anchor = None
        self.last_fix_stamp = None
        self.mission = Mission()
        self.history = collections.OrderedDict()
        self.events = collections.deque(maxlen=60)
        self.traj_seq = 0
        self.goal_stamp = float("inf")
        self.accepted_trajectory = None
        self.last_forwarded=time.monotonic()
        self.fence_violation=None
        self.controller = None
        self.controller_log = None
        self.stopping = False
        self.goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.command_pub = rospy.Publisher('/ground_station/position_cmd', PositionCommand, queue_size=1)
        rospy.Subscriber('/position_cmd', PositionCommand, self.cmd_cb, queue_size=1)
        rospy.Subscriber('/mavros/state', State, lambda m: self.store('state',m))
        rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, lambda m: self.store('setpoint',m),queue_size=1)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, lambda m: self.store('extended',m))
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_cb, queue_size=5)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, lambda m:self.store('gps',m),queue_size=5)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, lambda m:self.store('heading',m))
        rospy.Subscriber('/mavros/battery', BatteryState, lambda m:self.store('battery',m))
        rospy.Subscriber('/cloud_registered', PointCloud2, lambda m:self.store('cloud',m),queue_size=1)
        rospy.Subscriber('/drone_%d_fastlio/bridge_status'%(aircraft-1), String, lambda m:self.store('bridge',m))
        rospy.Subscriber('/drone_0_planning/bspline', Bspline, self.traj_cb, queue_size=5)
        self.log('观测代理已启动；未启动控制器、未解锁')

    def log(self, text):
        self.events.append(dict(time=time.strftime('%H:%M:%S'),text=text))
        self.rospy.loginfo(text)

    def store(self, key, msg):
        with self.data_lock:
            self.data[key] = (msg,time.monotonic())

    def odom_cb(self, msg):
        with self.data_lock:
            self.store('odom',msg)
            self.odom_buffer.append((msg,time.monotonic()))

    def traj_cb(self, msg):
        with self.data_lock:
            self.traj_seq += 1
            self.store('traj',msg)
            if msg.start_time.to_sec() >= self.goal_stamp:
                if self.mission.fence:
                    try:
                        if msg.order!=3 or len(msg.pos_pts)<4:raise ValueError('不支持的规划轨迹结构')
                        if len(msg.knots)!=len(msg.pos_pts)+4 or not all(math.isfinite(k) for k in msg.knots) or any(a>b for a,b in zip(msg.knots,msg.knots[1:])):raise ValueError('规划轨迹节点向量无效')
                        # A cubic B-spline lies within each four-point control hull.
                        for i in range(len(msg.pos_pts)-3):
                            if not self.mission.fence.control_hull([(p.x,p.y) for p in msg.pos_pts[i:i+4]]):
                                raise ValueError('规划轨迹控制包络越过内缩边界')
                    except (ValueError,TypeError) as e:
                        self.accepted_trajectory=None;self.fence_violation=str(e);return
                self.accepted_trajectory = msg.traj_id

    def cmd_cb(self, msg):
        # Do not replay a previous mission while taking off for the next one.
        if (self.mission.phase in {"OUTBOUND", "RETURNING"}
                and self.accepted_trajectory == msg.trajectory_id
                and -.1 <= (self.rospy.Time.now()-msg.header.stamp).to_sec() <= .3):
            fence=self.mission.fence
            if fence:
                try:
                    p=[msg.position.x,msg.position.y];q=[p[0]+msg.velocity.x,p[1]+msg.velocity.y]
                    if not fence.segment(p,q):raise ValueError('跟踪指令将进入边界缓冲区')
                except (ValueError,TypeError) as e:self.fence_violation=str(e);return
            self.command_pub.publish(msg)
            self.last_forwarded=time.monotonic()

    def snapshot(self):
        with self.data_lock:
            return self._snapshot()

    def _snapshot(self):
        now = time.monotonic()
        def get(key, age):
            pair = self.data.get(key)
            return pair[0] if pair and now-pair[1] <= age else None
        state, ext, odom = get('state',2), get('extended',3), get('odom',.8)
        pos, speed, yaw = None, None, None
        velocity=None
        if odom:
            p,q,v = odom.pose.pose.position,odom.pose.pose.orientation,odom.twist.twist.linear
            values = [p.x,p.y,p.z,v.x,v.y,v.z,q.x,q.y,q.z,q.w]
            norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            age = (self.rospy.Time.now()-odom.header.stamp).to_sec()
            if all(math.isfinite(a) for a in values) and norm>.5 and -.1<=age<=.8:
                pos=[p.x,p.y,p.z];velocity=[v.x,v.y,v.z];speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),norm-2*(q.y*q.y+q.z*q.z))
        bridge = get('bridge',2)
        try:
            bridge_ready = bool(bridge and json.loads(bridge.data).get('ready'))
        except (ValueError,TypeError):
            bridge_ready=False
        cloud=get('cloud',2)
        points=cloud.width*cloud.height if cloud else 0
        gps=get('gps',2); heading=get('heading',1)
        gps_valid=False
        gps_info=None
        if gps:
            age=(self.rospy.Time.now()-gps.header.stamp).to_sec()
            gps_valid=(gps.status.status>=0 and all(math.isfinite(v) for v in [gps.latitude,gps.longitude,gps.altitude])
                       and -90<=gps.latitude<=90 and -180<=gps.longitude<=180 and -.1<=age<=2)
            if gps_valid:
                gps_info=dict(latitude=gps.latitude,longitude=gps.longitude,altitude=gps.altitude)
        # Capture only while stationary and disarmed; pair GNSS with timestamp-matched odometry.
        if state and not state.armed and not self.mission.active:
            if not gps_valid:
                self.anchor=None;self.samples.clear()
            elif heading and math.isfinite(heading.data) and pos and speed<.2:
                stamp=gps.header.stamp.to_sec()
                paired=[m for m,t in self.odom_buffer if now-t<2 and abs(m.header.stamp.to_sec()-stamp)<=.15]
                if paired and stamp!=self.last_fix_stamp:
                    paired_odom=min(paired,key=lambda m:abs(m.header.stamp.to_sec()-stamp))
                    q=paired_odom.pose.pose.orientation;p=paired_odom.pose.pose.position
                    n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
                    local_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),n-2*(q.y*q.y+q.z*q.z))
                    rotation=local_yaw-(math.pi/2-math.radians(heading.data))
                    candidate=dict(lat=gps.latitude,lon=gps.longitude,alt=gps.altitude,x=p.x,y=p.y,rotation=rotation)
                    if self.samples:
                        old=self.samples[0]
                        xy=to_local(candidate['lat'],candidate['lon'],old)
                        angle=math.atan2(math.sin(rotation-old['rotation']),math.cos(rotation-old['rotation']))
                        if math.hypot(xy[0]-p.x,xy[1]-p.y)>.5 or abs(angle)>.08:
                            self.samples.clear();self.anchor=None
                    self.samples.append(candidate);self.last_fix_stamp=stamp
                    if len(self.samples)>=5:
                        self.anchor=candidate
            else:
                self.anchor=None;self.samples.clear()
        nodes=self.data.get('nodes',(set(),0))
        node_set=nodes[0] if now-nodes[1]<3 else set()
        fresh=bool(state and ext and pos)
        planner=bool(self.goal.get_num_connections()>0 and '/drone_0_traj_server' in node_set)
        ready=bool(fresh and state.connected and bridge_ready and points>0 and planner)
        reasons=[]
        for condition,label in [(state and state.connected,'飞控未连接/状态过期'),(pos,'局部定位过期/无效'),(ext,'着陆状态未知'),(bridge_ready,'定位桥未就绪'),(points,'点云无更新'),(planner,'规划器未就绪')]:
            if not condition:reasons.append(label)
        batt=get('battery',5)
        battery=None
        if batt and math.isfinite(batt.percentage) and 0<=batt.percentage<=1:
            battery=round(batt.percentage*100,1)
        return dict(aircraft=self.aircraft,version=VERSION,session_id=self.session_id,connected=bool(state and state.connected),fresh=fresh,
                    ready=ready,reasons=reasons,armed=bool(state and state.armed),landed=bool(ext and ext.landed_state==1),
                    mode=state.mode if state else 'UNKNOWN',position=pos,speed=speed,yaw=yaw,battery=battery,
                    voltage=batt.voltage if batt and math.isfinite(batt.voltage) else None,
                    capabilities=['fence-v1','flight-v2'],geo_anchor=dict(self.anchor) if self.anchor else None,velocity=velocity,
                    command_age=max(0.,now-self.last_forwarded),
                    gps=gps_info,geo_ready=bool(gps_valid and self.anchor and heading and pos),
                    bridge_ready=bridge_ready,cloud_points=points,controller='/px4_controller' in node_set,
                    traj_seq=self.traj_seq,mission=self.mission.status(),events=list(self.events),stopping=self.stopping)

    def service(self, name, kind, **kwargs):
        # Bounded wait; never automatically repeat arm/mode requests after an uncertain result.
        self.rospy.wait_for_service(name, timeout=3)
        result=[]
        def call():
            try:result.append(self.rospy.ServiceProxy(name,kind)(**kwargs))
            except Exception as e:result.append(e)
        worker=threading.Thread(target=call,daemon=True);worker.start();worker.join(4)
        if not result:raise RuntimeError('服务响应超时，结果未知：'+name)
        if isinstance(result[0],Exception):raise result[0]
        return result[0]

    def perform(self, actions):
        from mavros_msgs.srv import CommandBool, SetMode
        from geometry_msgs.msg import PoseStamped
        for action,value in actions:
            if action=='log':self.log(value)
            elif action=='controller_start':
                self.controller_log=open('/tmp/ground-station-controller.log','a')
                self.controller=subprocess.Popen(['rosrun','controller','px4_controller_node','__name:=px4_controller',
                     '_position_cmd_topic:=/ground_station/position_cmd'],stdout=self.controller_log,stderr=subprocess.STDOUT,start_new_session=True)
                time.sleep(2.2)
                if self.controller.poll() is not None:raise RuntimeError('控制器启动失败')
            elif action=='arm':
                s=self.snapshot()
                if not s['ready'] or s['armed'] or not s['landed']:raise ValueError('解锁前复检失败')
                fault=self.mission.fence_fault(s)
                if fault:raise ValueError(fault)
                with self.data_lock:
                    sp=self.data.get('setpoint')
                if not sp or time.monotonic()-sp[1]>.3:
                    raise RuntimeError('控制器设定值未持续输出，拒绝解锁')
                if not self.service('/mavros/set_mode',SetMode,base_mode=0,custom_mode='OFFBOARD').mode_sent:
                    raise RuntimeError('飞控拒绝 OFFBOARD，未解锁')
                deadline=time.monotonic()+3
                while self.snapshot()['mode']!='OFFBOARD' and time.monotonic()<deadline:
                    time.sleep(.1)
                if self.snapshot()['mode']!='OFFBOARD':raise RuntimeError('OFFBOARD 未确认，未解锁')
                if not self.service('/mavros/cmd/arming',CommandBool,value=True).success:
                    raise RuntimeError('飞控拒绝解锁')
            elif action=='goal':
                msg=PoseStamped();msg.header.stamp=self.rospy.Time.now();msg.header.frame_id='map'
                self.goal_stamp=msg.header.stamp.to_sec();self.accepted_trajectory=None
                self.last_forwarded=time.monotonic()
                msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=value;msg.pose.orientation.w=1
                self.goal.publish(msg)
                self.log('规划目标 x=%.2f y=%.2f z=%.2f'%tuple(value))
            elif action=='land':
                if not self.service('/mavros/set_mode',SetMode,base_mode=0,custom_mode='AUTO.LAND').mode_sent:
                    raise RuntimeError('飞控未接受 AUTO.LAND，请手动接管')
                self.log('已请求 AUTO.LAND，等待飞控状态确认')
            elif action=='controller_stop':
                if self.controller and self.controller.poll() is None:
                    os.killpg(self.controller.pid,signal.SIGINT)
                    try:self.controller.wait(timeout=3)
                    except subprocess.TimeoutExpired:self.log('控制器退出等待超时')
                self.controller=None
                if self.controller_log:self.controller_log.close();self.controller_log=None

    def failure(self, exc):
        self.log(str(exc))
        s=self.snapshot()
        if s['fresh'] and not s['armed']:
            self.mission.transition('ERROR',time.monotonic(),str(exc))
            self.perform([('controller_stop',None)])
        else:
            self.mission.transition('LAND_REQUESTED',time.monotonic(),str(exc))
            try:self.perform([('land',None)])
            except Exception as e:self.log('降落请求异常：'+str(e))

    def command(self, req):
        with self.lock:
            command=req.get('command')
            if command=='status':return dict(ok=True,status=self.snapshot())
            token=req.get('id')
            if not isinstance(token,str) or not 8<=len(token)<=80:raise ValueError('请求缺少唯一 id')
            if token in self.history:return self.history[token]
            if self.stopping:raise ValueError('代理已锁定为停止状态，请重启程序')
            s=self.snapshot();now=time.monotonic()
            # Validation failures have no effects. Action failures are recorded; never retry implicitly.
            if command in {'prepare','start'}:
                plan=req.get('flight_plan')
                if not isinstance(plan,dict):raise ValueError('需要地面站生成的围栏航线，请更新地面站')
                if not isinstance(plan.get('local_frame'),dict):raise ValueError('航线缺少起飞坐标参考，请重启新版地面站并重新搜索')
                reference=plan.get('geo_anchor')
                if plan.get('mode')=='competition' and not reference:raise ValueError('比赛任务缺少地理参考')
                if reference:
                    if not s['geo_ready']:raise ValueError('地理参考失效，请重新搜索')
                    mapped=to_local(reference['lat'],reference['lon'],self.anchor)
                    angle=math.atan2(math.sin(reference['rotation']-self.anchor['rotation']),math.cos(reference['rotation']-self.anchor['rotation']))
                    if math.dist(mapped,[reference['x'],reference['y']])>.25 or abs(angle)>.005:raise ValueError('地理参考已变化，请重新搜索航线')
                points=[]
                for p in req.get('waypoints',[]):
                    if p.get('kind')=='geo':
                        if not s['geo_ready']:raise ValueError('无有效经纬度/同步参考，请使用米制航点')
                        points.append(to_local(p['a'],p['b'],self.anchor))
                    elif p.get('kind')=='local':points.append([p['a'],p['b']])
                    else:raise ValueError('未知航点类型')
                if command=='prepare':
                    Mission().start(points,s,now,plan)
                    response=dict(ok=True,status=self.snapshot(),resolved_points=points)
                    self.history[token]=response
                    return response
                actions=self.mission.start(points,s,now,plan)
                self.fence_violation=None
            elif command=='return':actions=self.mission.return_home(s,now)
            elif command=='land':actions=self.mission.land(s,now)
            elif command=='stop':
                if not s['fresh'] or s['armed'] or not s['landed'] or self.mission.active:
                    raise ValueError('关闭要求新鲜的已着陆、未解锁状态且没有执行中的任务')
                self.stopping=True;actions=[('controller_stop',None)]
            else:raise ValueError('未知命令')
            try:
                self.log('收到操作：'+command)
                self.perform(actions)
                response=dict(ok=True,status=self.snapshot())
            except Exception as e:
                self.failure(e)
                response=dict(ok=False,error=str(e),status=self.snapshot())
            self.history[token]=response
            while len(self.history)>200:self.history.popitem(last=False)
            return response

    def run(self):
        import rosgraph
        last_nodes=0
        while not self.rospy.is_shutdown():
            now=time.monotonic()
            if now-last_nodes>1:
                try:
                    graph=rosgraph.Master('/ground_station_agent').getSystemState()
                    nodes={node for section in graph for topic,names in section for node in names}
                    self.store('nodes',nodes)
                except Exception as e:self.rospy.logwarn_throttle(5,str(e))
                last_nodes=now
            with self.lock:
                before=self.mission.phase
                try:
                    if self.fence_violation and self.mission.phase in {'TAKEOFF','OUTBOUND','RETURNING'}:
                        fault=self.fence_violation;self.fence_violation=None
                        state=self.snapshot()
                        if state.get('mode')=='OFFBOARD':self.perform(self.mission.land(state,now,fault))
                        else:self.perform(self.mission.tick(state,now))
                    else:self.perform(self.mission.tick(self.snapshot(),now))
                except Exception as e:self.failure(e)
                if before!=self.mission.phase:self.log('任务状态：'+self.mission.phase+' '+self.mission.reason)
            time.sleep(.1)


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads=True


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(25)
        try:
            line=self.rfile.readline(65537)
            if len(line)>65536:raise ValueError('请求过大')
            reply=self.server.agent.command(json.loads(line))
        except Exception as e:reply=dict(ok=False,error=str(e))
        self.wfile.write(json.dumps(reply,allow_nan=False).encode()+b'\n')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['serve','request'])
    parser.add_argument('--aircraft',type=int,choices=range(1,8))
    args=parser.parse_args()
    if args.action=='request':
        import sys
        print(json.dumps(request(json.load(sys.stdin)),allow_nan=False))
        return
    if args.aircraft is None:parser.error('--aircraft required')
    lock=open('/tmp/fast-drone-ground-station.lock','w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('agent already running')
    import rospy
    rospy.init_node('ground_station_agent',disable_signals=False)
    agent=Agent(args.aircraft)
    if os.path.exists(SOCKET):os.unlink(SOCKET)
    with Server(SOCKET,Handler) as server:
        os.chmod(SOCKET,0o600)
        server.agent=agent
        threading.Thread(target=server.serve_forever,daemon=True).start()
        try:agent.run()
        finally:server.shutdown()


if __name__=='__main__':main()
