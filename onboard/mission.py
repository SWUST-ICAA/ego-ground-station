"""Deterministic per-aircraft mission; actions executed by the ROS adapter."""
import math
try:
    from .fence import Fence, xy
except ImportError:
    from fence import Fence, xy

TERMINAL = {'IDLE', 'COMPLETE', 'ERROR', 'MANUAL'}


class Mission:
    def __init__(self):
        self.phase = 'IDLE'
        self.reason = ''
        self.points = []
        self.index = 0
        self.home = None
        self.since = 0.
        self.dwell = None
        self.traj_base = 0
        self.seen_offboard = False
        self.fence=None;self.return_points=[];self.return_index=0;self.visited=[];self.requires_geo=False
        self.local_frame=None;self.skipped=[];self.progress_position=None;self.progress_time=0.

    @property
    def active(self):
        return self.phase not in TERMINAL

    def transition(self, phase, now, reason=''):
        self.phase, self.since, self.reason, self.dwell = phase, now, reason, None

    def start(self, points, state, now, plan=None):
        if self.active:
            raise ValueError('任务正在执行，请先返航或降落')
        if not state.get('ready') or not state.get('fresh') or not state.get('connected'):
            raise ValueError('本机定位/飞控/点云/规划器尚未就绪')
        if state.get('armed') or not state.get('landed') or state.get('controller'):
            raise ValueError('起飞要求已着陆、未解锁且无其他控制器')
        if state.get('speed', 100) > .2:
            raise ValueError('飞机尚未静止')
        pos = [float(x) for x in state['position']]
        if len(pos) != 3 or not all(math.isfinite(x) for x in pos):
            raise ValueError('本机位置无效')
        validated = []
        if not isinstance(points, list) or not 1 <= len(points) <= 50:
            raise ValueError('航点数量应为 1～50')
        for point in points:
            if len(point) != 2:
                raise ValueError('每个航点需要 x/y')
            x, y = map(float, point)
            if not math.isfinite(x+y) or not abs(x) < 498.7 or not abs(y) < 498.7:
                raise ValueError('航点超出当前地图边界（|x|<498.7m，|y|<498.7m）')
            validated.append([x, y, 1.5])
        if abs(pos[0]) >= 498.7 or abs(pos[1]) >= 498.7:
            raise ValueError('起飞点超出规划地图')
        fence=None;returns=[]
        if plan is not None:
            frame=plan.get('local_frame')
            if frame:
                values=[float(frame[k]) for k in ('x','y','yaw')]
                if not all(math.isfinite(v) for v in values):raise ValueError('起飞坐标参考无效')
                yaw=state.get('yaw')
                if yaw is None or abs(math.atan2(math.sin(yaw-values[2]),math.cos(yaw-values[2])))>.05:
                    raise ValueError('机头方向已变化，请重新搜索航线')
            if plan.get('mode') not in {'test','competition'}:raise ValueError('围栏模式无效')
            fence=Fence(plan['polygon'],plan['margin'])
            if math.dist(pos[:2],xy(plan['origin']))>.3:raise ValueError('起点已变化，请重新搜索航线')
            last=pos[:2]
            for point in validated:
                if not fence.segment(last,point[:2],.3):raise ValueError('去程航线超出围栏或余量不足')
                last=point[:2]
            raw=plan['return_points']
            if not 1<=len(raw)<=50:raise ValueError('返航途经点数量无效')
            for point in raw:
                if not fence.segment(last,point,.3):raise ValueError('返航航线超出围栏或余量不足')
                returns.append([float(point[0]),float(point[1]),1.5]);last=point
            if math.dist(last,pos[:2])>.3:raise ValueError('返航终点与起飞点不一致')
            previous=returns[-2][:2] if len(returns)>1 else validated[-1][:2]
            if not fence.segment(previous,pos[:2],.3):raise ValueError('起飞点不在返航安全区')
            returns[-1]=[pos[0],pos[1],1.5]
        self.fence=fence;self.return_points=returns;self.return_index=0
        self.local_frame=dict(plan['local_frame']) if plan and plan.get('local_frame') else None
        self.skipped=[]
        self.requires_geo=bool(plan and (plan.get('mode')=='competition' or plan.get('geo_anchor')))
        self.visited=[[pos[0],pos[1],1.5]]
        self.points, self.home, self.index = validated, [pos[0], pos[1], 1.5], 0
        self.seen_offboard = False
        self.transition('ARMING', now)
        return [('controller_start', None), ('arm', None)]

    def send_goal(self, point, state, now, phase):
        self.transition(phase, now)
        self.traj_base = state.get('traj_seq', 0)
        self.progress_position=list(state['position']);self.progress_time=now
        return [('goal', point)]

    def skip_blocked(self,state,now,reason):
        """Advance in route order; leave path feasibility to the onboard planner."""
        phase=self.phase;idx=self.index if phase=='OUTBOUND' else self.return_index
        self.skipped.append(dict(phase=phase,index=idx,reason=reason))
        route=self.points if phase=='OUTBOUND' else (self.return_points or [self.home])
        prefix=('去程' if phase=='OUTBOUND' else '返航')+f'第 {idx+1} 点：{reason}'
        for j in range(idx+1,len(route)):
            if phase=='OUTBOUND':self.index=j
            else:self.return_index=j
            actions=self.send_goal(route[j],state,now,phase)
            self.reason=prefix+f'；跳至第 {j+1} 点'
            return [('log',self.reason)]+actions
        if phase=='OUTBOUND':
            self.return_index=0
            actions=self.send_goal(self.return_points[0] if self.return_points else self.home,state,now,'RETURNING')
            self.reason=prefix+'；去程点已处理完，进入返航'
            return [('log',self.reason)]+actions
        return [('log',prefix+'；返航路径无法继续')]+self.land(state,now,'返航航点持续不可达，请求就地降落')

    def land(self, state, now, reason='操作员请求降落'):
        if not state.get('fresh'):
            raise ValueError('飞控状态过期，无法确认降落请求条件')
        if not state.get('armed'):
            raise ValueError('飞机未解锁，无需降落')
        self.transition('LAND_REQUESTED', now, reason)
        return [('land', None)]

    def return_home(self, state, now):
        if self.home is None or not state.get('ready') or not state.get('fresh'):
            raise ValueError('无有效起飞点或本机定位不可用')
        if self.phase not in {'OUTBOUND', 'RETURNING'} or state.get('mode') != 'OFFBOARD':
            raise ValueError('仅在本次自主巡航中可以规划返航；其他状态请降落或手动接管')
        if self.phase=='RETURNING':return []
        if self.fence:
            fault=self.fence_fault(state)
            if fault:return self.land(state,now,fault)
            if math.dist(state['position'],self.home)<=.3 and state.get('speed',100)<=.2:return self.land(state,now,'已在起飞点上方，请求降落')
            # Follow reached outbound corners backwards; never shortcut a concave boundary.
            candidates=list(reversed(self.visited))
            first=next((i for i,p in enumerate(candidates) if self.fence.segment(state['position'][:2],p[:2])),None)
            if first is None:return self.land(state,now,'当前位置不能安全接入返航途经点，请求就地降落')
            self.return_points=candidates[first:];self.return_index=0
            return self.send_goal(self.return_points[0],state,now,'RETURNING')
        return self.send_goal(self.home, state, now, 'RETURNING')

    def fence_fault(self,state):
        if not self.fence:return ''
        pos=state.get('position')
        if not pos or not self.fence.contains(pos[:2]):return '实际位置已进入边界缓冲区'
        if self.requires_geo and not state.get('geo_ready'):return '比赛/经纬度任务的地理参考失效'
        velocity=state.get('velocity') or [0.,0.,0.]
        predicted=[pos[i]+velocity[i] for i in (0,1)]
        if not self.fence.segment(pos[:2],predicted):return '按当前速度预计 1 秒内进入边界缓冲区'
        return ''

    def stable(self, state, point, now):
        near = math.dist(state['position'], point) <= .3 and state.get('speed', 100) <= .2
        if not near:
            self.dwell = None
            return False
        if self.dwell is None:
            self.dwell = now
        return now-self.dwell >= 1.5

    def tick(self, state, now):
        if not self.active:
            return []
        fresh = state.get('fresh', False)
        if self.phase in {'LAND_REQUESTED', 'LANDING'}:
            if fresh and state.get('landed') and not state.get('armed'):
                self.transition('COMPLETE', now, self.reason)
                return [('controller_stop', None)]
            if self.phase == 'LAND_REQUESTED':
                if fresh and state.get('mode') == 'AUTO.LAND':
                    self.transition('LANDING', now, self.reason)
                    return [('controller_stop', None)]
                if now-self.since > 8:
                    self.transition('ERROR', now, '降落模式未确认，请检查飞控/手动接管；未强制锁桨')
            elif fresh and state.get('armed') and state.get('mode') != 'AUTO.LAND':
                self.transition('MANUAL', now, '降落被外部模式接管')
            elif now-self.since > 90:
                self.transition('ERROR', now, '降落/自动锁定超时，请检查飞控状态')
            return []
        if self.phase == 'ARMING':
            if fresh and state.get('armed'):
                self.transition('TAKEOFF', now)
            elif now-self.since > 12:
                if not fresh:
                    self.transition('LAND_REQUESTED', now, '解锁结果未知且遥测过期，请求降落')
                    return [('land', None)]
                self.transition('ERROR', now, '解锁未确认')
                return [('controller_stop', None)]
            return []
        if fresh and not state.get('armed'):
            self.transition('ERROR', now, '任务中飞控已锁定，任务取消')
            return [('controller_stop', None)]
        if fresh and (self.phase in {'OUTBOUND', 'RETURNING'} or (self.phase=='TAKEOFF' and self.seen_offboard)) and state.get('mode') != 'OFFBOARD':
            self.transition('MANUAL', now, '飞行模式已改变，停止任务，不自动抢回控制')
            return [('controller_stop', None)]
        if not fresh or not state.get('ready'):
            self.transition('LAND_REQUESTED', now, '本机定位/传感器/飞控数据失效，请求就地降落')
            return [('land', None)]
        fault=self.fence_fault(state)
        if fault:
            self.transition('LAND_REQUESTED',now,fault);return [('land',None)]
        if self.phase == 'TAKEOFF':
            if self.seen_offboard and state.get('mode') != 'OFFBOARD':
                self.transition('MANUAL', now, '起飞过程中已切换模式，停止自动任务')
                return [('controller_stop', None)]
            if state.get('mode') == 'OFFBOARD':
                self.seen_offboard = True
            if state.get('mode') == 'OFFBOARD' and self.stable(state, self.home, now):
                return self.send_goal(self.points[0], state, now, 'OUTBOUND')
            if now-self.since > 30:
                self.transition('LAND_REQUESTED', now, '起飞超时，请求降落')
                return [('land', None)]
            return []
        if self.phase in {'OUTBOUND', 'RETURNING'}:
            target = (self.return_points[self.return_index] if self.return_points else self.home) if self.phase == 'RETURNING' else self.points[self.index]
            if self.progress_position is None or math.dist(state['position'],self.progress_position)>.3:
                self.progress_position=list(state['position']);self.progress_time=now
            if state.get('traj_seq', 0) > self.traj_base and self.stable(state, target, now):
                if self.phase == 'RETURNING':
                    if self.return_points and self.return_index+1<len(self.return_points):
                        self.return_index+=1;return self.send_goal(self.return_points[self.return_index],state,now,'RETURNING')
                    self.transition('LAND_REQUESTED', now, '已返回起飞点，自动降落')
                    return [('land', None)]
                self.visited.append(list(self.points[self.index]))
                self.index += 1
                if self.index == len(self.points):
                    self.return_index=0
                    return self.send_goal(self.return_points[0] if self.return_points else self.home, state, now, 'RETURNING')
                return self.send_goal(self.points[self.index], state, now, 'OUTBOUND')
            reason=''
            if now-self.since>3 and state.get('command_age',0)>3:reason='有效跟踪指令中断超过 3 秒'
            elif now-self.progress_time>10 and math.dist(state['position'],target)>.45:reason='连续 10 秒位置无进展'
            elif now-self.since>120:reason='当前航点执行超过 120 秒'
            elif now-self.since>12 and state.get('traj_seq',0)<=self.traj_base:reason='规划器未生成轨迹'
            if reason:return self.skip_blocked(state,now,reason)
        return []

    def status(self):
        return dict(phase=self.phase, reason=self.reason, points=self.points,local_frame=self.local_frame,skipped=list(self.skipped),
                    index=self.index, home=self.home, active=self.active, return_points=self.return_points,
                    return_index=self.return_index, fence=dict(polygon=self.fence.polygon,margin=self.fence.margin) if self.fence else None)
