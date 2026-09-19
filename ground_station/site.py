"""Site profiles and ground-side routes. Competition coordinates from subject 3."""
import copy
import math
from onboard.fence import Fence
from onboard.geo import to_local

# Latitude, longitude in decimal degrees; datum must be confirmed by operator.
COMPETITION=[[(33+51/60+s/3600),(113+42/60+t/3600)] for t,s in
             [(27.74,56.63),(28.09,55.53),(29.29,55.36),(28.36,45.94),
              (30.85,42.99),(35.32,42.86),(36.36,56.12),(29.29,56.53)]]
DEFAULT=dict(mode='test',margin=3.,datum_confirmed=False,polygon=[[-20,-20],[20,-20],[20,20],[-20,20]])


def build_plan(profile,targets,state):
    profile=copy.deepcopy(profile)
    if not state.get('position') or not state.get('fresh'):raise ValueError('需要新鲜本机定位才能搜索航线')
    if state.get('armed') or not state.get('landed') or state.get('mission',{}).get('active'):raise ValueError('请在着陆、未解锁且无任务时生成航线')
    if 'fence-v1' not in state.get('capabilities',[]):raise ValueError('机载代理尚不支持围栏，请更新代理后再执行')
    anchor=state.get('geo_anchor')
    uses_geo=profile['mode']=='competition' or any(p['kind']=='geo' for p in targets)
    if uses_geo and (not state.get('geo_ready') or not anchor):raise ValueError('需要有效 GNSS 和坐标参考')
    if profile['mode']=='competition':
        if not profile.get('datum_confirmed'):raise ValueError('尚未确认赛事边界为 WGS84，请先核实坐标系')
        polygon=[to_local(lat,lon,anchor) for lat,lon in COMPETITION]
    elif profile['mode']=='test':polygon=profile['polygon']
    else:raise ValueError('未知场地模式')
    fence=Fence(polygon,profile['margin']);start=list(state['position'][:2]);last=start;points=[]
    if not targets:raise ValueError('请先设置目标点')
    for p in targets:
        goal=to_local(p['a'],p['b'],anchor) if p['kind']=='geo' else [p['a'],p['b']]
        points.extend(fence.route(last,goal));last=goal
    returns=fence.route(last,start)
    if len(points)>50 or len(returns)>50:raise ValueError('搜索结果超过 50 个途经点，请减少目标或简化场地')
    return dict(waypoints=[dict(kind='local',a=p[0],b=p[1]) for p in points],
                flight_plan=dict(polygon=polygon,margin=fence.margin,mode=profile['mode'],
                                 origin=start,return_points=returns,geo_anchor=anchor if uses_geo else None),
                preview=dict(polygon=polygon,margin=fence.margin,outbound=[start]+points,return_path=[last]+returns))
