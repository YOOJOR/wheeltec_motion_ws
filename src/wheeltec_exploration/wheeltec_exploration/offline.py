"""Deterministic virtual RGB-D room; ALL pose updates here are simulated, no ROS I/O."""
import argparse
from collections import Counter
import json
from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np
from wheeltec_rgbd.posed import PosedObservation, rigid, transform
from .core import Config, Explorer
from .upstream import load_fmm, LOCKED_COMMIT
from .runtime import Recorder


def virtual_observation(x=0.,y=0.,yaw=0.,stamp=1):
    base=transform([x,y,0],[0,0,np.sin(yaw/2),np.cos(yaw/2)])
    optical=np.array([[0,0,1,.15],[-1,0,0,0],[0,-1,0,.45],[0,0,0,1.]])
    camera=rigid(base @ optical)
    height,width=48,64
    k=np.array([[45.,0,31.5],[0,45.,23.5],[0,0,1]])
    v,u=np.mgrid[:height,:width]
    rays=np.stack(((u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1],np.ones_like(u)),axis=-1)
    direction=rays @ camera[:3,:3].T
    origin=camera[:3,3]
    candidates=[]
    with np.errstate(divide='ignore',invalid='ignore'):
        floor=-origin[2]/direction[...,2]
        candidates.append(np.where((floor>0)&(direction[...,2]<0),floor,np.inf))
        for axis in (0,1):
            for wall in (-3.,3.):
                depth=(wall-origin[axis])/direction[...,axis]
                candidates.append(np.where(depth>0,depth,np.inf))
    depth=np.min(candidates,axis=0).astype(np.float32)
    depth[(depth<.4)|(depth>2.)]=np.nan
    obs=SimpleNamespace(stamp_ns=stamp,depth_stamp_ns=stamp,color_stamp_ns=stamp,
        depth_m=depth,rgb=np.full((height,width,3),160,np.uint8),
        depth_info=SimpleNamespace(k=k.ravel().tolist()),
        calibration_source='synthetic_room')
    return PosedObservation(obs,stamp,base,camera)


def simulate(fmm_class, steps=200, seed=1):
    engine=Explorer(Config(pixel_stride=2,seed=seed),fmm_class)
    x=y=yaw=0.
    trajectory=[]
    actions=Counter()
    reasons=Counter()
    timings=[]
    initial=None
    goals=set()
    for i in range(steps):
        stamp=1_000_000_000+i*100_000_000
        posed=virtual_observation(x,y,yaw,stamp)
        begin=time.monotonic()
        decision=engine.update(posed,i*.1)
        timings.append(time.monotonic()-begin)
        actions[decision.action]+=1
        reasons[decision.reason]+=1
        if initial is None: initial=decision.as_dict()
        if decision.goal_xy: goals.add(tuple(decision.goal_xy))
        trajectory.append([x,y,yaw])
        # Pure numerical kinematics of a virtual robot, never a publisher.
        yaw+=decision.angular_z*.1
        next_x=x+decision.linear_x*np.cos(yaw)*.1
        next_y=y+decision.linear_x*np.sin(yaw)*.1
        if max(abs(next_x),abs(next_y)) <= 2.65: x,y=next_x,next_y
    report={'source':'synthetic_rgbd_room','motion_output':False,'vlm_enabled':False,
        'upstream_commit':LOCKED_COMMIT,'steps':steps,'seed':seed,'actions':dict(actions),
        'reasons':dict(reasons),'unique_goals':len(goals),'first_decision':initial,
        'last_decision':decision.as_dict(),
        'simulated_displacement_m':float(np.hypot(x,y)),
        'processing_mean_sec':float(np.mean(timings)), 'processing_max_sec':max(timings),
        'map_cells':int(engine.grid.free.sum()) if engine.grid else 0,
        'obstacle_cells':int(engine.grid.obstacle.sum()) if engine.grid else 0,
        'real_camera_or_pose_used':False}
    return report,engine,np.asarray(trajectory)


def main(args=None):
    parser=argparse.ArgumentParser(description='Offline virtual room, no ROS nodes or hardware')
    parser.add_argument('--upstream-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--steps',type=int,default=200)
    parser.add_argument('--seed',type=int,default=1)
    opts=parser.parse_args(args)
    if not 1<=opts.steps<=2000: parser.error('steps must be 1..2000')
    report,engine,trajectory=simulate(load_fmm(opts.upstream_root),opts.steps,opts.seed)
    Recorder(opts.output).save(report,engine.grid)
    np.save(opts.output/'simulated_trajectory.npy',trajectory)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    if report['reasons'].get('PLANNED',0)==0: raise SystemExit('No valid plan in virtual room')

if __name__ == '__main__':
    main()
