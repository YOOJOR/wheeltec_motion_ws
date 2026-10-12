"""Paper frontier geometry, measured scan and bounded upstream-style fallback."""
import ast
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from skimage import measure
from wheeltec_exploration.core import Config, Grid, Explorer
from wheeltec_exploration.offline import virtual_observation, simulate
from wheeltec_exploration.upstream import load_fmm

UPSTREAM = Path('/home/wheeltec/workspace/conavgpt2_ws/upstream/Co-NavGPT2')


@pytest.fixture(scope='module')
def fmm(): return load_fmm(UPSTREAM)


@pytest.mark.parametrize('values',[{'exploration_policy':'other'},{'initial_scan_angle':-1},
    {'initial_scan_angle':7},{'initial_scan_angle':float('nan')},{'initial_scan_timeout':0},
    {'paper_frontier_min_cells':0},{'fallback_sample_attempts':0},{'fallback_sample_attempts':1001}])
def test_invalid_policy_config(values):
    with pytest.raises(ValueError): Config(**values)


def test_explored_layer_projects_points_without_marking_unseen_rays_observed():
    grid=Grid(Config(),[0,0])
    grid.update(np.array([[1.,0,.5],[0.,1.,2.]]),[.15,0,.1157],[0,0,0])
    assert grid.explored[grid.cell([1.,0])]
    assert grid.explored[grid.cell([0.,1.])]
    assert grid.obstacle[grid.cell([1.,0])]
    assert not grid.obstacle[grid.cell([0.,1.])]
    assert grid.free[grid.cell([.5,0])]
    assert not grid.explored[grid.cell([.5,0])]
    assert grid.occupancy()[grid.cell([.5,0])]==-1


@pytest.mark.parametrize('scene',['rectangle','obstacle_split','small_and_large'])
def test_frontiers_match_locked_upstream_global_extractor(scene):
    # Extract only the pure method. Never import upstream Go2/Agent/VLM modules.
    tree=ast.parse((UPSTREAM/'utils/explored_map_utils.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Global_Map_Proc')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='Frontier_Det')
    namespace={'np':np,'cv2':cv2,'measure':measure}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<upstream-frontier>','exec'),namespace)
    grid=Grid(Config(),[0,0])
    grid.explored[90:145,90:160]=True
    if scene=='obstacle_split':grid.obstacle[85:150,125:128]=True
    if scene=='small_and_large':grid.explored[20:28,20:28]=True
    source=SimpleNamespace(explored_map=grid.explored.astype(float),obstacle_map=grid.obstacle.astype(float))
    _,_,expected=namespace['Frontier_Det'](source,threshold_point=20)
    actual,_=grid.paper_frontiers()
    assert actual==[tuple(v) for v in expected]


def test_unknown_is_available_to_planner_but_remains_unknown_on_map(fmm):
    engine=Explorer(Config(initial_scan_angle=0.),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    before=engine.grid.occupancy().copy()
    decision=engine.plan(np.eye(4),0.)
    assert decision.reason=='PLANNED'
    assert decision.goal_source=='random_map_fallback' and decision.planning_allows_unknown
    assert decision.frontier_count==0
    assert engine.grid.occupancy()[engine.goal]==-1
    np.testing.assert_array_equal(engine.grid.occupancy(),before)


def test_frontier_precedes_fallback_and_has_no_old_045m_filter(fmm):
    engine=Explorer(Config(initial_scan_angle=0.),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    engine.plan(np.eye(4),0.)
    assert engine.goal_kind=='random_map_fallback'
    # Rectangle centroid near the robot survives paper extraction; no 0.45 m veto.
    engine.grid.explored[114:128,120:134]=True
    goals,_=engine.grid.paper_frontiers()
    assert goals and np.linalg.norm(engine.grid.xy(goals[0]))<.45
    engine.plan(np.eye(4),.1)
    assert engine.goal_kind=='random_frontier'


def test_fallback_is_bounded_and_does_not_force_obstacles_or_disconnected_space_free(fmm):
    engine=Explorer(Config(initial_scan_angle=0.,fallback_sample_attempts=5),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    engine.grid.obstacle[:,150]=True
    decision=engine.plan(np.eye(4),0.)
    assert decision.reason=='PLANNED' and engine.goal[1]<144
    assert engine.grid.reachable(engine.grid.cell([0,0]),allow_unknown=True)[engine.goal]
    class BlockedRNG:
        calls=0
        def random(self,n):self.calls+=1;return np.array([.9,.9])
    engine.goal=None;engine.rng=BlockedRNG()
    assert engine.plan(np.eye(4),.1).reason=='NO_VALID_RANDOM_GOAL'
    assert engine.rng.calls==5
    assert engine.grid.obstacle[:,150].all()


def test_already_arrived_frontier_does_not_trap_planner_in_reached_loop(fmm):
    engine=Explorer(Config(initial_scan_angle=0.),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    engine.grid.explored[110:131,110:131]=True
    goals,_=engine.grid.paper_frontiers()
    assert goals and np.linalg.norm(engine.grid.xy(goals[0]))<.25
    decision=engine.plan(np.eye(4),0.)
    assert decision.reason=='PLANNED' and decision.goal_source=='random_map_fallback'


def test_scan_requires_measured_rotation_wraps_pi_and_never_counts_old_frames(fmm):
    engine=Explorer(Config(initial_scan_angle=.6),fmm)
    first=virtual_observation(yaw=3.,stamp=1)
    assert engine.update(first,0.).reason=='INITIAL_SCAN'
    assert engine.update(first,.1).reason=='NO_NEW_OBSERVATION'
    assert engine.scan_progress==0
    for i,yaw in enumerate([3.2,3.4,3.61],2):
        decision=engine.update(virtual_observation(yaw=yaw,stamp=i),i*.1)
    assert engine.scan_complete and engine.scan_progress==pytest.approx(.61)
    assert decision.reason!='INITIAL_SCAN'


def test_stationary_or_reverse_turn_does_not_complete_scan_and_timeout_latches(fmm):
    engine=Explorer(Config(initial_scan_angle=.5,initial_scan_timeout=1.),fmm)
    for i,yaw in enumerate([0.,0.,-.2]):
        result=engine.update(virtual_observation(yaw=yaw,stamp=i+1),i*.2)
        assert result.reason=='INITIAL_SCAN' and result.linear_x==0 and result.angular_z==.25
    assert not engine.scan_complete and engine.scan_progress<0
    result=engine.update(virtual_observation(stamp=4),1.1)
    assert result.reason=='INITIAL_SCAN_TIMEOUT' and result.linear_x==result.angular_z==0
    assert engine.grid is None
    assert engine.update(virtual_observation(stamp=5),1.2).reason=='INITIAL_SCAN_TIMEOUT'
    engine.reset()
    assert engine.scan_progress==0 and engine.scan_started is None
    assert engine.update(virtual_observation(stamp=6),1.3).reason=='INITIAL_SCAN'


def test_scan_stops_on_nearby_obstacle(fmm):
    engine=Explorer(Config(),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    engine.grid.obstacle[engine.grid.cell([.2,0])]=True
    result=engine.initial_scan(np.eye(4),0.)
    assert result.reason=='INITIAL_SCAN_BLOCKED' and result.angular_z==0


def test_full_virtual_scan_then_exploration_without_hardware(fmm):
    report,engine,trajectory=simulate(fmm,280,1)
    assert report['reasons']['INITIAL_SCAN']>200
    assert report['reasons'].get('PLANNED',0)>0
    assert engine.scan_complete and engine.scan_progress>=2*np.pi
    assert trajectory[:,2].max()>=2*np.pi  # Later planning may legitimately turn back.
    assert not report['motion_output'] and not report['real_camera_or_pose_used']
