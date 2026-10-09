"""Shared flight configuration, host persistence, and ROS readback. No flight actions."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

DEFAULTS = dict(map_size_x=1000., map_size_y=1000., map_size_z=.4,
                ground_height=1., obstacles_inflation=1., resolution=.2,
                max_acc=3., max_vel=1.2, takeoff_height=1.2)
NAMESPACE = '/drone_0_ego_planner_node/'
ROS_KEYS = {key: 'grid_map/'+key for key in DEFAULTS if key not in {'max_acc','max_vel','takeoff_height'}}
ROS_KEYS.update(max_acc='manager/max_acc', max_vel='manager/max_vel', takeoff_height='fsm/waypoint_height')
LAUNCH_DIR = Path('src/planner/plan_manage/launch')


def voxel_count(p):
    return math.prod(math.ceil(p['map_size_'+axis]/p['resolution']) for axis in 'xyz')


def validate(values, max_inflation_cells=10):
    if not isinstance(values, dict) or set(values) != set(DEFAULTS):
        raise ValueError('必须提供完整的 9 项参数')
    p = {}
    for key, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError('参数必须为有限数值：'+key)
        p[key] = float(value)
    for key in set(DEFAULTS)-{'ground_height'}:
        if p[key] <= 0: raise ValueError('参数必须大于 0：'+key)
    if p['resolution'] < .02: raise ValueError('地图分辨率不得小于 0.02 m')
    if math.ceil((p['obstacles_inflation']-1e-5)/p['resolution']) > max_inflation_cells:
        raise ValueError('膨胀距离最多为 %d 个分辨率单元，请减小膨胀距离或增大分辨率'%max_inflation_cells)
    if min(p['map_size_x'],p['map_size_y']) <= 2*(p['obstacles_inflation']+.3):
        raise ValueError('地图 XY 尺寸必须大于两侧膨胀距离及边界余量')
    if p['map_size_z'] < 2*p['resolution']-1e-8:
        raise ValueError('地图 Z 尺寸至少需要两个分辨率单元')
    if not p['ground_height']+1e-4 < p['takeoff_height'] < p['ground_height']+p['map_size_z']-1e-4:
        raise ValueError('起飞高度必须位于地图高度范围内部（地面高度～地面高度+Z）')
    if voxel_count(p) > 100_000_000:
        raise ValueError('地图超过 1 亿网格（基础缓冲区约 1.5 GB），请增大分辨率或缩小尺寸')
    return p


def same(a, b):
    return set(a)==set(b) and all(math.isclose(a[k],b[k],rel_tol=1e-8,abs_tol=1e-8) for k in a)


def map_limits(p, rolling=False):
    margin=p['obstacles_inflation']+.3
    # A rolling local map is a sensor window, not a fixed world boundary.
    extent=DEFAULTS if rolling else p
    x,y=[extent['map_size_'+axis]/2-margin for axis in 'xy']
    return (-x,x,-y,y)


def diff_map_geometry(p):
    # Match the fine-resolution GridMap without silently enlarging resolution.
    resolution=p['resolution']
    half_z=max(2.,p['map_size_z']/2.)
    cells=[2*math.ceil(v/resolution) for v in (p['map_size_x']/2.,p['map_size_y']/2.,half_z)]
    inflation=math.ceil((p['obstacles_inflation']-1e-5)/resolution)
    voxels=math.prod(cells)
    return dict(resolution=resolution,size=[v*resolution for v in cells],voxels=voxels,
                buffer_bytes=26*voxels+4*math.prod(v+2*inflation for v in cells))


def validate_diff(values):
    p=validate(values,max_inflation_cells=32)
    if diff_map_geometry(p)['buffer_bytes']>1_000_000_000:
        raise ValueError('滚动地图基础缓冲区超过 1 GB，请缩小窗口或增大分辨率')
    return p


def _diff_trees(root):
    return [ET.parse(Path(root)/'deploy'/name,parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
            for name in ('planner.launch','planner_params.xml')]


def _read_diff(root, aircraft):
    single,advanced=_diff_trees(root)
    arg=lambda key:float(_element(single,'arg',key).get('value'))
    p=dict(map_size_x=2*arg('local_map_size_x'),map_size_y=2*arg('local_map_size_y'),
           map_size_z=arg('virtual_ceil')-arg('virtual_ground'),ground_height=arg('virtual_ground'),
           obstacles_inflation=arg('inflation_size'),resolution=arg('map_resolution'),
           max_acc=arg('max_acc'),max_vel=arg('max_vel'),
           takeoff_height=float(_element(advanced,'param','fsm/waypoint_height').get('value')))
    if not math.isclose(arg('local_map_size_z'),max(2.,p['map_size_z']/2.)):
        raise ValueError('滚动地图 Z 窗口与高度范围不一致')
    saved=Path(root)/'deploy/aircraft'/str(int(aircraft))/'flight_parameters.json'
    if not same(validate_diff(json.loads(saved.read_text())),p):
        raise ValueError('保存配置与启动文件不一致，请检查机上配置备份')
    return validate_diff(p)


def _write_diff(root, aircraft, values):
    p=validate_diff(values);single,advanced=_diff_trees(root)
    args=dict(local_map_size_x=p['map_size_x']/2.,local_map_size_y=p['map_size_y']/2.,
              local_map_size_z=max(2.,p['map_size_z']/2.),virtual_ground=p['ground_height'],
              virtual_ceil=p['ground_height']+p['map_size_z'],inflation_size=p['obstacles_inflation'],
              map_resolution=p['resolution'],max_acc=p['max_acc'],max_vel=p['max_vel'])
    for i in range(5):args['point%d_z'%i]=p['takeoff_height']
    for key,value in args.items():_element(single,'arg',key).set('value',str(value))
    _element(advanced,'param','fsm/waypoint_height').set('value',str(p['takeoff_height']))
    _element(advanced,'param','grid_map/visualization_truncate_height').set('value',str(args['virtual_ceil']))
    paths=[root/'deploy'/name for name in ('planner.launch','planner_params.xml')]
    return _persist(root,aircraft,p,paths,(single,advanced))


def _trees(root):
    # Keep existing launch comments and settings unrelated to this feature.
    parser = lambda: ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return [ET.parse(Path(root)/LAUNCH_DIR/name, parser=parser())
            for name in ('single_run_in_exp.launch','advanced_param_exp.xml')]


def _element(tree, tag, name):
    found = [e for e in tree.iter(tag) if e.get('name')==name]
    if len(found)!=1: raise ValueError('启动文件参数缺失或重复：'+name)
    return found[0]


def read_files(root, aircraft):
    if (Path(root)/'deploy/planner.launch').is_file():return _read_diff(root,aircraft)
    single, advanced = _trees(root)
    p = dict(DEFAULTS)
    for key in ('map_size_x','map_size_y','map_size_z','max_acc','max_vel'):
        p[key] = float(_element(single,'arg',key).get('value'))
    for key in ('ground_height','obstacles_inflation','resolution'):
        p[key] = float(_element(advanced,'param','grid_map/'+key).get('value'))
    height = [e for e in advanced.iter('param') if e.get('name')=='fsm/waypoint_height']
    if height: p['takeoff_height']=float(height[0].get('value'))
    saved = Path(root)/'deploy/aircraft'/str(int(aircraft))/'flight_parameters.json'
    if saved.exists() and not same(validate(json.loads(saved.read_text())),p):
        raise ValueError('保存配置与启动文件不一致，请检查机上配置备份')
    return validate(p)


def _atomic(path, data):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        with temp.open('wb') as stream:
            stream.write(data);stream.flush();os.fsync(stream.fileno())
        os.replace(temp,path)
    finally:
        if temp.exists():temp.unlink()


def write_files(root, aircraft, values):
    root=Path(root)
    if (root/'deploy/planner.launch').is_file():return _write_diff(root,aircraft,values)
    p=validate(values)
    single,advanced=_trees(root)
    for key in ('map_size_x','map_size_y','map_size_z','max_acc','max_vel'):
        _element(single,'arg',key).set('value',str(p[key]))
    for i in range(5):_element(single,'arg','point%d_z'%i).set('value',str(p['takeoff_height']))
    for key in ('ground_height','obstacles_inflation','resolution'):
        _element(advanced,'param','grid_map/'+key).set('value',str(p[key]))
    # Keep the occupancy visualization available throughout the selected Z range.
    _element(advanced,'param','grid_map/visualization_truncate_height').set('value',str(p['ground_height']+p['map_size_z']))
    heights=[e for e in advanced.iter('param') if e.get('name')=='fsm/waypoint_height']
    if not heights:
        nodes=list(advanced.iter('node'))
        if len(nodes)!=1:raise ValueError('无法定位规划器节点')
        heights=[ET.SubElement(nodes[0],'param',name='fsm/waypoint_height',type='double')]
    heights[0].set('value',str(p['takeoff_height']))
    paths=[root/LAUNCH_DIR/name for name in ('single_run_in_exp.launch','advanced_param_exp.xml')]
    return _persist(root,aircraft,p,paths,(single,advanced))


def _persist(root,aircraft,p,paths,trees):
    paths.append(root/'deploy/aircraft'/str(int(aircraft))/'flight_parameters.json')
    data=[ET.tostring(t.getroot(),encoding='utf-8')+b'\n' for t in trees]
    data.append((json.dumps(p,indent=2,allow_nan=False)+'\n').encode())
    backup=root/'deploy/parameter-backups'/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    originals=[path.read_bytes() if path.exists() else None for path in paths]
    for path,old in zip(paths,originals):
        if old is not None:_atomic(backup/path.relative_to(root),old)
    try:
        for path,content in zip(paths,data):_atomic(path,content)
        if not same(read_files(root,aircraft),p):raise ValueError('保存后的配置回读不一致')
    except Exception:
        for path,old in zip(paths,originals):
            if old is None:
                if path.exists():path.unlink()
            else:_atomic(path,old)
        raise
    return str(backup)


def runtime_parameters(rospy):
    if rospy.get_param('/navigation_stack','') == 'diff-planner-px4':
        get=lambda name:float(rospy.get_param(NAMESPACE+name))
        floor=get('grid_map/virtual_ground')
        p=validate_diff(dict(map_size_x=2*get('grid_map/local_update_range_x'),
                             map_size_y=2*get('grid_map/local_update_range_y'),
                             map_size_z=get('grid_map/virtual_ceil')-floor,
                             ground_height=floor,
                             obstacles_inflation=get('grid_map/obstacles_inflation'),
                             resolution=get('grid_map/resolution'),
                             max_acc=get('manager/max_acc'),max_vel=get('manager/max_vel'),
                             takeoff_height=get('fsm/waypoint_height')))
        if rospy.get_param(NAMESPACE+'grid_map/fine_resolution_version',0)!=1:
            raise ValueError('规划器镜像尚未支持真实细分辨率，请更新镜像')
        if not math.isclose(p['resolution'],get('grid_map/actual_resolution')):
            raise ValueError('规划器实际分辨率与配置不一致')
        for quantity in ('max_acc','max_vel'):
            if not math.isclose(p[quantity],get('optimization/'+quantity)):
                raise ValueError('规划器速度/加速度参数不一致：'+quantity)
        if not math.isclose(get('grid_map/local_update_range_z'),max(2.,p['map_size_z']/2.)):
            raise ValueError('滚动地图 Z 窗口与高度范围不一致')
        return p
    p=validate({k:rospy.get_param(NAMESPACE+v,DEFAULTS[k]) for k,v in ROS_KEYS.items()})
    for quantity,other in (('max_vel','optimization/max_vel'),('max_vel','bspline/limit_vel'),
                           ('max_acc','optimization/max_acc'),('max_acc','bspline/limit_acc')):
        if not math.isclose(float(rospy.get_param(NAMESPACE+other,p[quantity])),p[quantity]):
            raise ValueError('规划器速度/加速度参数不一致：'+other)
    return p


def main():
    parser=argparse.ArgumentParser();parser.add_argument('command',choices=['read','write'])
    parser.add_argument('--root',required=True);parser.add_argument('--aircraft',type=int,required=True)
    parser.add_argument('--container',default='fast-drone-250')
    args=parser.parse_args()
    lock=Path(args.root)/'deploy/.flight-parameters.lock'
    with lock.open('a') as stream:
        fcntl.flock(stream,fcntl.LOCK_EX)
        backup=None
        if args.command=='write':
            running=subprocess.check_output(['docker','inspect','-f','{{.State.Running}}',args.container],text=True).strip()
            if running!='false':raise ValueError('保存要求先关闭机上程序')
            backup=write_files(args.root,args.aircraft,json.load(sys.stdin))
        print(json.dumps(dict(values=read_files(args.root,args.aircraft),backup=backup),allow_nan=False))


if __name__=='__main__':main()
