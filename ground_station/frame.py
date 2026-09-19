"""Ground UI coordinates: right/forward relative to a frozen takeoff reference."""
import math
import copy


def reference(state):
    p=state.get('position');yaw=state.get('yaw')
    if not p or yaw is None or not all(math.isfinite(float(v)) for v in (*p,yaw)):
        raise ValueError('需要有效位置和机头方向才能建立起飞坐标')
    return dict(x=float(p[0]),y=float(p[1]),yaw=float(yaw))


def to_map(point,ref):
    x,y=point[:2];s,c=math.sin(ref['yaw']),math.cos(ref['yaw'])
    return [ref['x']+s*x+c*y,ref['y']-c*x+s*y]


def from_map(point,ref):
    dx,dy=point[0]-ref['x'],point[1]-ref['y'];s,c=math.sin(ref['yaw']),math.cos(ref['yaw'])
    return [s*dx-c*dy,c*dx+s*dy]+list(point[2:])


def display(state,points,trace,preview,ref):
    state=copy.deepcopy(state);preview=copy.deepcopy(preview)
    if not ref:return state,[],[],None
    if state.get('position'):state['position']=from_map(state['position'],ref)
    mission=state.get('mission',{})
    if mission.get('home'):mission['home']=from_map(mission['home'],ref)
    mission['points']=[from_map(p,ref) for p in mission.get('points',[])]
    if mission.get('fence'):mission['fence']['polygon']=[from_map(p,ref) for p in mission['fence']['polygon']]
    if preview:
        for key in ('polygon','outbound','return_path'):preview[key]=[from_map(p,ref) for p in preview.get(key,[])]
    targets=[]
    for point in points:
        if point['kind']!='local':continue
        p=dict(point)
        if p.get('frame')!='right-forward':p['a'],p['b']=from_map([p['a'],p['b']],ref)
        targets.append(p)
    return state,targets,[tuple(from_map(p,ref)) for p in trace],preview
