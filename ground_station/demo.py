"""Offline demonstrator: uses the same mission logic; never opens SSH."""
import time
import math
from onboard.mission import Mission
from onboard.geo import to_local


class Demo:
    def __init__(self,config):
        self.config=config;self.mission=Mission();self.target=[0.,0.,0.];self.enabled=False;self.last=time.monotonic()
        self.s=dict(aircraft=config['id'],program=False,connected=False,ready=False,fresh=True,armed=False,
                    landed=True,position=[0.,0.,0.],speed=0.,mode='AUTO.LOITER',controller=False,traj_seq=0,
                    battery=92.,voltage=24.1,bridge_ready=True,cloud_points=180,geo_ready=config['id']!=2,
                    yaw=0.,command_age=0.,capabilities=['fence-v1','flight-v2','planner-view-v1'],geo_anchor=dict(lat=30.,lon=104.,alt=500.,x=0.,y=0.,rotation=0.) if config['id']!=2 else None,
                    gps=dict(latitude=30.,longitude=104.,altitude=500.) if config['id']!=2 else None,events=[],reasons=[])
    def close(self):pass
    def observe(self):
        return dict(position=self.s['position'],inflated=[],trajectory=[],map_age_sec=None,
                    traj_age_sec=None,map_frame='world',source_points=0)
    def actions(self,actions):
        for action,val in actions:
            if action=='controller_start':self.s['controller']=True
            elif action=='arm':self.s.update(armed=True,landed=False,mode='OFFBOARD');self.target=[0.,0.,1.5]
            elif action=='goal':self.target=list(val);self.s['traj_seq']+=1
            elif action=='land':self.s['mode']='AUTO.LAND';self.target=[*self.s['position'][:2],0.]
            elif action=='controller_stop':self.s['controller']=False
    def status(self):
        now=time.monotonic();dt=min(now-self.last,.5);self.last=now
        if self.s['armed']:
            dist=math.dist(self.s['position'],self.target);step=min(dist,dt*1.2)
            if dist>1e-6:self.s['position']=[a+(b-a)*step/dist for a,b in zip(self.s['position'],self.target)]
            self.s['speed']=step/max(dt,.001)
            if self.s['mode']=='AUTO.LAND' and self.s['position'][2]<.02:self.s.update(armed=False,landed=True,speed=0.)
        self.actions(self.mission.tick(self.s,now))
        return dict(self.s,mission=self.mission.status())
    def execute(self,command,**params):
        now=time.monotonic()
        if command=='start_program':self.s.update(program=True,ready=True,connected=True)
        elif command=='stop_program':
            if self.s['armed'] or self.mission.active:raise ValueError('飞行中禁止关闭程序')
            self.s.update(program=False,ready=False,connected=False)
        elif command=='start':
            points=[]
            for p in params['waypoints']:
                if p['kind']=='geo':
                    if not self.s['geo_ready']:raise ValueError('无经纬度，米制可用')
                    points.append(to_local(p['a'],p['b'],dict(lat=30,lon=104,alt=500,x=0,y=0,rotation=0)))
                else:points.append([p['a'],p['b']])
            self.actions(self.mission.start(points,self.s,now,params.get('flight_plan')))
        elif command=='return':self.actions(self.mission.return_home(self.s,now))
        elif command=='land':self.actions(self.mission.land(self.s,now))
        return self.status()
