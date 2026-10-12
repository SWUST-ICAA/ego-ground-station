import ast
import math
import sys
import time
import threading
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from onboard.mission import Mission
DIFF_DEFAULTS = dict(map_size_x=11., map_size_y=11., map_size_z=1.2,
                     ground_height=1., obstacles_inflation=.9, resolution=.05,
                     max_acc=3., max_vel=1.3, takeoff_height=1.2)


def state(**changes):
    s = dict(ready=True, fresh=True, connected=True, armed=True, landed=False,
             mode='OFFBOARD', position=[0., 0., 1.2], speed=0., velocity=[0., 0., 0.],
             yaw=1.5, traj_seq=1, command_age=8.)
    s.update(changes)
    return s


def mission():
    m = Mission(DIFF_DEFAULTS, rolling_map=True)
    m.exploration_enabled = True
    m.points = [[0., 5., 1.2], [0., 20., 1.2]]
    m.index = 1
    m.home = [0., 0., 1.2]
    m.phase = 'OUTBOUND'
    m.since = 0.
    m.goal_timeout = 120.
    m.progress_position = [0., 0., 1.2]
    m.progress_time = 0.
    m.fence = NS(polygon=[[-10., -10.], [30., -10.], [30., 30.], [-10., 30.]],
                 margin=1., contains=lambda p: True, segment=lambda a,b: True)
    return m


def test_exploration_keeps_final_goal_when_no_path_or_command():
    m = mission()
    for now in [5., 20., 200.]:
        assert m.tick(state(), now) == []
        assert m.phase == 'OUTBOUND'
        assert m.index == 1
        assert not m.skipped


def test_exploration_sensor_failure_still_requests_land():
    m = mission()
    assert m.tick(state(ready=False, reasons=['定位桥未就绪']), 5.) == [('land', None)]
    assert m.phase == 'LAND_REQUESTED'


def test_exploration_failed_landing_is_not_completion():
    m = mission()
    m.land(state(), 5., '操作员请求降落')
    assert m.tick(state(armed=False, landed=True), 10.) == [('controller_stop', None)]
    assert m.phase == 'ABORTED'


def test_ordinary_mission_retains_command_loss_behavior():
    m = mission()
    m.exploration_enabled = False
    assert m.tick(state(), 5.)
    assert m.phase == 'RETURNING'


def grid(stamp=100., seq=1, blocked=False):
    data = [0] * 400
    if blocked: data[10*20+11] = 100
    return NS(header=NS(frame_id='map', stamp=NS(to_sec=lambda: stamp), seq=seq),
              info=NS(width=20, height=20, resolution=.5,
                      origin=NS(position=NS(x=-5., y=-5., z=1.2),
                                orientation=NS(x=0., y=0., z=0., w=1.))), data=data)


def session():
    from onboard.exploration_session import ExplorationSession
    e = ExplorationSession(DIFF_DEFAULTS)
    e.reset('mission-one', 99.)
    e.context(mission(), 100.)
    e.update_grid(grid(), 10.)
    p = dict(stamp=100., ready=True, session='mission-one', target_id='OUTBOUND:1',
             reject_id=0, goal=[1., 0., 1.2], map_seq=1, map_stamp=100., mode='explore')
    e.update_proposal(p, 10.)
    return e, p


def test_context_contains_final_task_and_fence():
    e, _ = session()
    c = e.context(mission(), 100.)
    assert c['active'] and c['target'] == [0., 20., 1.2]
    assert c['phase'] == 'OUTBOUND' and c['target_id'] == 'OUTBOUND:1'
    assert c['height'] == 1.2 and c['clearance'] == .9
    assert c['polygon'] == mission().fence.polygon


def test_accepts_only_fresh_matching_safe_local_proposal():
    e, p = session()
    assert e.proposal_goal(mission(), [0., 0., 1.2], 100.1, 10.1) == [1., 0., 1.2]
    assert e.proposal_goal(mission(), [0., 0., 1.2], 102., 12.) is None
    for field, value in [('session','old'), ('target_id','RETURNING:0'), ('reject_id',4),
                         ('ready',False), ('goal',[float('nan'),0.,1.2]),
                         ('goal',[1.,0.,2.]), ('goal',[10.,0.,1.2])]:
        q = dict(p); q[field] = value
        e.update_proposal(q, 10.)
        assert e.proposal_goal(mission(), [0., 0., 1.2], 100.1, 10.1) is None


def test_remaining_trajectory_stops_on_new_obstacle_and_stale_grid():
    e, _ = session()
    hull = [[i/5.,0.,1.2] for i in range(6)]
    assert e.validate_hulls([hull], [2.], 0., 100.1, 10.1)
    e.update_grid(grid(stamp=100.1, seq=2, blocked=True), 10.1)
    assert not e.validate_hulls([hull], [2.], 0., 100.2, 10.2)
    assert not e.validate_hulls([hull], [2.], 0., 103., 13.)


def test_start_chooses_final_outbound_target_for_exploration():
    m = Mission(DIFF_DEFAULTS, rolling_map=True)
    m.exploration_enabled = True
    m.start([[0.,5.], [0.,20.]], state(armed=False, landed=True, controller=False), 0.)
    assert m.index == 1


def test_transport_uploads_exploration_dependencies():
    tree = ast.parse((ROOT/'ground_station/transport.py').read_text())
    names = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant)]
    assert names.count('exploration_session.py') >= 2
    assert names.count('exploration_geometry.py') >= 2


def agent():
    sys.path.insert(0, str(ROOT/'onboard'))
    from agent import Agent
    a = Agent.__new__(Agent)
    a.exploration_enabled = True
    a.explorer, _ = session()
    a.mission = mission()
    a.diff_planner = True
    a.controller_kind = 'px4'
    a.data_lock = threading.RLock()
    a.data = {}
    a.traj_seq = 0
    a.goal_stamp = 100.
    a.accepted_trajectory = None
    a.accepted_hulls = None
    a.local_goal = [1.,0.,1.2]
    a.local_goal_since = 10.
    a.fence_violation = None
    a.last_forwarded = 10.
    a.last_reject = 0.
    a.hold_yaw = 1.5
    a.messages = []
    a.command_pub = NS(publish=lambda m: a.messages.append(m))
    a.log = lambda m: None
    a.snapshot = lambda: state()
    class Stamp:
        def __init__(self, value): self.value = value
        def to_sec(self): return self.value
        def __sub__(self, other): return Stamp(self.value-other.to_sec())
    a.rospy = NS(Time=NS(now=lambda: Stamp(100.1)))
    a.mission.fence.control_hull = lambda h: True
    a.explorer.update_grid(grid(), time.monotonic())
    return a


def trajectory(moving=True):
    return NS(order=5, duration=[2.], coef_x=[0.,0.,0.,0.,.5 if moving else 0.,0.],
              coef_y=[0.]*6, coef_z=[0.,0.,0.,0.,0.,1.2], traj_id=3,
              start_time=NS(to_sec=lambda: 100.))


def test_agent_rejects_emergency_hover_for_distant_local_goal():
    a = agent()
    a.traj_cb(trajectory(False))
    assert a.accepted_trajectory is None
    assert a.fence_violation is None


def test_agent_rechecks_remaining_curve_before_forwarding():
    a = agent()
    a.traj_cb(trajectory())
    assert a.accepted_trajectory == 3
    a.explorer.update_grid(grid(stamp=100.1, seq=2, blocked=True), time.monotonic())
    cmd = NS(trajectory_id=3, header=NS(stamp=a.rospy.Time.now()),
             position=NS(x=.05,y=0.,z=1.2),velocity=NS(x=.5,y=0.,z=0.))
    a.cmd_cb(cmd)
    assert a.accepted_trajectory is None
    assert not a.messages


def test_stationary_command_preserves_actual_yaw():
    a = agent()
    cmd = NS(velocity=NS(x=0.,y=0.,z=0.),yaw=0.,yaw_dot=0.)
    fixed = a.exploration_yaw(cmd)
    assert fixed.yaw == pytest.approx(1.5)
    assert fixed.yaw_dot == 0.
    assert cmd.yaw == 0.


def test_moving_command_retains_planned_yaw():
    a = agent()
    cmd = NS(velocity=NS(x=1.,y=0.,z=0.),yaw=.7,yaw_dot=.1)
    assert a.exploration_yaw(cmd).yaw == .7


def test_early_return_landing_does_not_claim_final_goal_completed():
    m = mission()
    m.returned_home = True
    m.phase = 'LANDING'
    m.tick(state(armed=False, landed=True), 10.)
    assert m.phase == 'ABORTED'


def test_reaching_final_target_then_home_completes_exploration():
    m = mission()
    s = state(position=[0.,20.,1.2], command_age=0.)
    m.tick(s, 2.)
    assert m.tick(s, 4.) == [('goal',m.home)]
    assert m.phase == 'RETURNING'
    s = state(traj_seq=2, command_age=0.)
    m.tick(s, 5.)
    assert m.tick(s, 7.) == [('land',None)]
    m.tick(state(armed=False, landed=True), 8.)
    assert m.phase == 'COMPLETE'


def test_final_goal_transition_and_context_read_are_atomic():
    a = agent()
    m = a.mission
    s = state(position=[0.,20.,1.2],command_age=0.)
    m.tick(s, 2.)
    attempted = threading.Event()
    results = []
    threads = []
    original = m.transition
    def read_context():
        attempted.set()
        try:
            with a.data_lock: results.append(a.explorer.context(m,100.))
        except Exception as exc: results.append(exc)
    def transition(phase, now, reason=''):
        if phase == 'RETURNING':
            t = threading.Thread(target=read_context)
            threads.append(t);t.start()
            assert attempted.wait(1)
        original(phase,now,reason)
    m.transition = transition
    a.perform = lambda actions: None
    a.tick_mission(s,4.)
    for t in threads: t.join(1)
    assert len(results)==1 and isinstance(results[0],dict)
    assert results[0]['phase']=='RETURNING'
