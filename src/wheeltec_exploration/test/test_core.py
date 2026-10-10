from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from wheeltec_rgbd.posed import rigid
from wheeltec_exploration.core import Config, Explorer, Grid, unproject, line
from wheeltec_exploration.configuration import load_configuration
from wheeltec_exploration.upstream import load_fmm
from wheeltec_exploration.offline import simulate, virtual_observation

UPSTREAM = Path('/home/wheeltec/workspace/conavgpt2_ws/upstream/Co-NavGPT2')


@pytest.fixture(scope='module')
def fmm(): return load_fmm(UPSTREAM)


@pytest.mark.parametrize('values',[{'forward_speed':.16},{'turn_speed':.36},{'resolution':0},
    {'size_m':1000},{'pixel_stride':0},{'seed':-1},{'clearance':float('nan')},
    {'obstacle_min_height':2.},{'goal_tolerance':1.}])
def test_invalid_or_unsafe_config(values):
    with pytest.raises(ValueError): Config(**values)


def test_projection_and_grid_axis_conventions():
    p=unproject(np.array([[1.,np.nan],[2.,0.]]),np.array([[1.,0,0],[0,1.,0],[0,0,1.]]),1)
    np.testing.assert_allclose(p,[[0,0,1],[0,2,2]])
    grid=Grid(Config(),[0,0])
    r,c=grid.cell([1.,2.])
    assert r>c
    assert np.linalg.norm(grid.xy((r,c))-[1.,2.]) < .05


def test_ray_order_preserved_both_directions():
    for a,b in [((10,4),(2,4)),((2,4),(10,4)),((5,5),(0,0))]:
        points=line(a,b)
        np.testing.assert_array_equal(points[0],a)
        np.testing.assert_array_equal(points[-1],b)


def test_occluding_obstacle_blocks_free_carving():
    grid=Grid(Config(pixel_stride=1),[0,0])
    grid.update(np.array([[.8,0,.5],[1.2,0,0]]),[.15,0,.5],[0,0,0])
    assert grid.obstacle[grid.cell([.8,0])]
    assert not grid.free[grid.cell([1.2,0])]
    assert grid.free[grid.cell([.4,0])]


def test_ceiling_is_not_proof_of_traversable_floor():
    grid=Grid(Config(),[0,0])
    grid.update(np.array([[1.,0,3.]]),[.15,0,.5],[0,0,0])
    assert not grid.free[grid.cell([1.,0])]


def test_unknown_and_disconnected_cells_excluded():
    grid=Grid(Config(),[0,0])
    base=grid.cell([0,0])
    grid.free[base[0]-10:base[0]+10,base[1]-10:base[1]+10]=True
    grid.free[2:8,2:8]=True
    reachable=grid.reachable(base)
    assert reachable[base] and not reachable[3,3] and not reachable[0,0]
    goals,_=grid.frontiers(reachable,base)
    assert all(reachable[g] for g in goals)


def test_obstacle_inflation_blocks_base():
    grid=Grid(Config(),[0,0])
    grid.free[:]=True
    grid.obstacle[grid.cell([.2,0])]=True
    assert not grid.reachable(grid.cell([0,0])).any()


def test_provisional_preview_height_is_explicitly_unconfirmed_and_d455_refused():
    path=Path(__file__).parents[1]/'config/astra_preview.yaml'
    cfg=load_configuration(path)
    assert cfg['camera']=='astra_s' and not cfg['extrinsics_confirmed']
    assert cfg['base_from_camera'][2,3] == .1157
    np.testing.assert_allclose(cfg['body_from_base'][:3,3],[-.10718,-.02237,-.20388])
    with pytest.raises(ValueError,match='NOT_CONFIGURED'): load_configuration(path,'d455')


def test_official_fmm_pipeline_is_deterministic_and_bounded(fmm):
    a,engine,trajectory=simulate(fmm,30,1)
    b,_,trajectory2=simulate(fmm,30,1)
    assert a['reasons'].get('PLANNED',0)>0
    assert a['actions']==b['actions']
    np.testing.assert_allclose(trajectory,trajectory2)
    assert engine.grid.free.shape==(240,240)
    assert not a['motion_output'] and not a['vlm_enabled']


def test_old_observation_not_processed_and_fault_requires_reset(fmm):
    engine=Explorer(Config(),fmm)
    posed=virtual_observation(stamp=1)
    engine.update(posed,0.)
    version=engine.grid.version
    assert engine.update(posed,.1).reason=='NO_NEW_OBSERVATION'
    assert engine.grid.version==version
    engine.halt('POSE_FAULT')
    assert engine.grid is None and engine.goal is None
    assert engine.update(virtual_observation(stamp=2),.2).reason=='POSE_FAULT'
    engine.reset()
    assert engine.update(virtual_observation(stamp=3),.3).reason!='POSE_FAULT'


def test_map_boundary_halts_without_wrapping(fmm):
    engine=Explorer(Config(),fmm)
    engine.update(virtual_observation(stamp=1),0.)
    result=engine.update(virtual_observation(x=20,stamp=2),.1)
    assert result.reason=='MAP_BOUNDARY' and engine.grid is None


def test_no_unknown_random_fallback_and_goal_stability(fmm):
    engine=Explorer(Config(),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    base=np.eye(4)
    assert engine.plan(base,0.).action=='stop'
    engine.grid.free[95:145,95:170]=True
    decision=engine.plan(base,.1)
    assert decision.reason=='PLANNED'
    goal=engine.goal
    for i in range(1,5):
        engine.plan(base,i*.1+.1)
        assert engine.goal==goal
    engine.plan(base,12.)
    assert goal in engine.blacklist


def test_turn_direction_and_speed_limits(fmm):
    engine=Explorer(Config(),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    engine.grid.free[95:145,95:170]=True
    # Same fixed target, opposite headings should command opposite turns.
    first=engine.plan(np.eye(4),0.)
    assert abs(first.linear_x)<=.15 and abs(first.angular_z)<=.35
    goal=engine.goal
    target=np.asarray(first.short_term_xy)
    desired=np.arctan2(target[1],target[0])
    for error,action in [(.8,'left'),(-.8,'right')]:
        yaw=desired-error
        base=np.eye(4)
        base[:2,:2]=[[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]
        assert engine.plan(base,.1).action==action
        assert engine.goal==goal


def test_semantic_target_approach_uses_known_reachable_floor_and_reset(fmm):
    engine=Explorer(Config(),fmm)
    engine.grid=Grid(engine.cfg,[0,0])
    engine.grid.free[100:141,100:181]=True
    engine.target_xy=np.array([2.,0.])
    decision=engine.plan(np.eye(4),0.)
    assert decision.reason=='PLANNED' and engine.goal_kind=='semantic_target'
    assert engine.grid.reachable(engine.grid.cell([0,0]))[engine.goal]
    base=np.eye(4)
    base[0,3]=1.5
    assert engine.plan(base,.1).reason=='TARGET_REACHED'
    engine.reset()
    assert engine.target_xy is None


def test_target_mask_uses_metres_and_correct_world_transform():
    from wheeltec_exploration.perception import target_points
    posed=virtual_observation(x=1.,stamp=3)
    mask=np.isfinite(posed.observation.depth_m)
    points=target_points(posed,mask)
    assert len(points)>0 and np.isfinite(points).all()
    assert points[:,0].mean()>1.
    with pytest.raises(ValueError):target_points(posed,np.zeros((2,2)))


def test_locked_fmm_grid_units_cannot_be_changed_silently():
    with pytest.raises(ValueError): Config(resolution=.1)
    with pytest.raises(ValueError): Config(goal_tolerance=.1)


def test_semantic_approach_does_not_switch_on_jitter_and_no_progress_is_reported(fmm):
    engine=Explorer(Config(),fmm)
    base=np.eye(4)
    engine.grid=Grid(engine.cfg,[0,0])
    engine.grid.free[100:141,100:181]=True
    engine.target_xy=np.array([2.,0.])
    assert engine.plan(base,.1).reason=='PLANNED'
    assert engine.goal_kind=='semantic_target'
    goal=engine.goal
    engine.target_xy+=np.array([.01,0.])
    engine.plan(base,.2)
    assert engine.goal==goal
    assert engine.plan(base,11.).reason=='TARGET_NO_PROGRESS'
    assert goal in engine.blacklist
    engine.plan(base,11.1)
    assert engine.goal != goal
