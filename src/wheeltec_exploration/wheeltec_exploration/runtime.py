"""Shadow-only runtime: no Twist publisher, navigation client, device startup or VLM."""
import argparse
from collections import deque
from dataclasses import replace
import copy
import json
from pathlib import Path
import signal
import threading
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wheeltec_rgbd.client import RGBDReader, NoFreshFrame
from wheeltec_rgbd.posed import PoseWindow, PosedObservation, transform, rigid
from wheeltec_rgbd.core import stamp_ns
from wheeltec_rgbd.node import latest_qos
from .configuration import load_configuration
from .core import Explorer, Decision
from .upstream import load_fmm, LOCKED_COMMIT


class PreviewNode(RGBDReader):
    def __init__(self, config, **kwargs):
        super().__init__(camera=config['camera'], **kwargs)
        self.config = config
        self.pose_lock = threading.Lock()
        self.poses = PoseWindow()
        self.reset_requested = threading.Event()
        self.odom_sub = self.create_subscription(Odometry,config['odom_topic'],self.receive_pose,latest_qos())
        prefix = config['prefix'].rstrip('/')
        self.decision_pub = self.create_publisher(String,prefix+'/decision',1)
        self.map_pub = self.create_publisher(OccupancyGrid,prefix+'/map',1)
        self.explored_pub = self.create_publisher(OccupancyGrid,prefix+'/explored_map',1)
        self.obstacle_pub = self.create_publisher(OccupancyGrid,prefix+'/obstacle_map',1)
        self.debug_pub = self.create_publisher(MarkerArray,prefix+'/debug_markers',1)
        self.goal_pub = self.create_publisher(PoseStamped,prefix+'/goal',1)
        self.stg_pub = self.create_publisher(PoseStamped,prefix+'/short_term_goal',1)
        self.reset_service = self.create_service(Trigger,prefix+'/reset',self.request_reset)

    def request_reset(self, request, response):
        self.reset_requested.set()
        response.success = True
        response.message = 'Reset queued; old map/task will be cleared. This program has no motion output.'
        return response

    def reset_inputs(self):
        with self.pose_lock:
            self.poses.reset()
            with self.lock:
                self.latest = None

    def receive_pose(self, msg):
        with self.pose_lock:
            try:
                if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
                    self.poses.fail('POSE_FRAME_MISMATCH')
                p,q = msg.pose.pose.position,msg.pose.pose.orientation
                matrix = transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
                self.poses.add(stamp_ns(msg),matrix,self.get_clock().now().nanoseconds,time.monotonic())
            except ValueError:
                self.poses.samples.clear()
                self.poses.reason = 'INVALID_POSE'
            except NoFreshFrame:
                pass

    def read_posed(self, last_stamp):
        with self.pose_lock:
            try:
                obs, received, epoch = self._read_latest_sample(last_stamp)
            except NoFreshFrame:
                self.poses.pending = None
                raise
            obs, stamp, wb = self.poses.match_observation(
                obs, received, epoch, last_stamp,
                self.get_clock().now().nanoseconds, time.monotonic(), self.settings)
            base = rigid(wb @ self.config['body_from_base'])
            return PosedObservation(obs,stamp,base,rigid(base @ self.config['base_from_camera']))

    def emit(self, decision, stamp, metrics):
        msg = String()
        msg.data = json.dumps({**decision.as_dict(), 'source_stamp_ns':stamp,
            'shadow':True,'motion_output':False,
            'selector':metrics.get('goal_kind',decision.goal_source),
            'initial_scan_skipped':self.config['map_config'].initial_scan_angle == 0,
            'camera':self.config['camera'],'world_frame':'camera_init',
            'extrinsics_confirmed':self.config['extrinsics_confirmed'],
            'extrinsics_source':self.config['extrinsics_source'], **metrics},ensure_ascii=False)
        self.decision_pub.publish(msg)
        for xy,pub in ((decision.goal_xy,self.goal_pub),(decision.short_term_xy,self.stg_pub)):
            if xy is None: continue
            pose = PoseStamped()
            pose.header.frame_id = 'camera_init'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x,pose.pose.position.y = map(float,xy)
            pose.pose.orientation.w = 1.
            pub.publish(pose)
        return json.loads(msg.data)

    def publish_grid(self, grid, floor_z=0.):
        msg = OccupancyGrid()
        msg.header.frame_id = 'camera_init'
        msg.header.stamp = self.get_clock().now().to_msg()
        if grid is not None:
            msg.info.resolution = grid.cfg.resolution
            msg.info.width = msg.info.height = grid.n
            msg.info.origin.position.x,msg.info.origin.position.y = map(float,grid.origin)
            msg.info.origin.position.z = float(floor_z)
            msg.info.origin.orientation.w = 1.
            msg.data = grid.occupancy().ravel().tolist()
        self.map_pub.publish(msg)
        explored,obstacles = copy.deepcopy(msg),copy.deepcopy(msg)
        if grid is not None:
            explored.data = np.where(grid.explored,0,-1).astype(np.int8).ravel().tolist()
            # Binary obstacle mask, not a free-space declaration.
            obstacles.data = (grid.obstacle.astype(np.int8)*100).ravel().tolist()
        self.explored_pub.publish(explored)
        self.obstacle_pub.publish(obstacles)

    def publish_debug(self, engine, report, floor_z=0.):
        array = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        array.markers.append(clear)
        def marker(kind, color, size):
            m = Marker(); m.header.frame_id = 'camera_init'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'exploration'; m.id = len(array.markers); m.type = kind
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = size
            m.color.r,m.color.g,m.color.b,m.color.a = (*color,1.)
            m.lifetime.nanosec = 500_000_000
            array.markers.append(m)
            return m
        grid = engine.grid
        if grid is not None:
            for cells,color,size in ((np.argwhere(engine.frontier_edge) if engine.frontier_edge is not None else [],
                                      (0.,.9,1.),.045),
                                     (engine.frontier_goals,(1.,.85,0.),.12)):
                m = marker(Marker.POINTS,color,size)
                for cell in cells:
                    x,y = grid.xy(cell)
                    m.points.append(Point(x=float(x),y=float(y),z=float(floor_z+.03)))
            if report.get('reason') == 'PLANNED':
                for name,color in (('goal_xy',(0.,1.,.2)),('short_term_xy',(1.,.5,0.))):
                    xy = report.get(name)
                    if xy is not None:
                        m = marker(Marker.SPHERE,color,.14)
                        m.pose.position = Point(x=float(xy[0]),y=float(xy[1]),z=float(floor_z+.08))
            m = marker(Marker.TEXT_VIEW_FACING,(1.,1.,1.),.14)
            center = grid.origin+grid.cfg.size_m/2
            m.pose.position = Point(x=float(center[0]),y=float(center[1]+.6),z=float(floor_z+.2))
            m.text = (f"{report.get('reason')} | {engine.goal_kind}\n"
                      f"scan {np.degrees(engine.scan_progress):.0f}/{np.degrees(engine.cfg.initial_scan_angle):.0f} deg (preview)")
        self.debug_pub.publish(array)


class Recorder:
    def __init__(self, directory):
        self.directory = Path(directory) if directory else None
        self.history = deque(maxlen=200)
        if self.directory: self.directory.mkdir(parents=True,exist_ok=True)

    def save(self, report, grid):
        self.history.append(report)
        if not self.directory: return
        target = self.directory/'report.json'
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        temporary.replace(target)
        (self.directory/'decisions.jsonl').write_text('\n'.join(json.dumps(v,ensure_ascii=False) for v in self.history)+'\n')
        if grid is None:
            for name in ('map.npz','map.png'):
                (self.directory/name).unlink(missing_ok=True)
            return
        np.savez_compressed(self.directory/'map.npz',occupancy=grid.occupancy(),
                            explored=grid.explored,obstacle=grid.obstacle,free=grid.free,
                            origin_xy=grid.origin,resolution=grid.cfg.resolution)
        import cv2
        occ=grid.occupancy()
        image=np.full((*occ.shape,3),128,np.uint8)
        image[occ==0]=240
        image[occ==100]=0
        cv2.imwrite(str(self.directory/'map.png'),np.flipud(image))


def run(node, engine, duration=0., output=None, perception=None):
    """Callbacks stay in a background executor; processing never blocks input spinning."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    start = time.monotonic()
    last_stamp,processed,last_good,last_save = 0,0,None,0.
    epoch = 0
    recorder = Recorder(output)
    perception_mode = 'local_models' if perception else 'geometry_only'
    report = {}
    floor_z = 0.
    try:
        while rclpy.ok() and (duration==0 or time.monotonic()-start < duration):
            loop = time.monotonic()
            if node.reset_requested.is_set():
                engine.reset()
                node.reset_inputs()
                node.reset_requested.clear()
                last_stamp,last_good = 0,None
                epoch += 1
                node.publish_grid(None)
            try:
                posed = node.read_posed(last_stamp)
                obs = posed.observation
                target_points, target_metrics = (perception.detect(posed) if perception else (None,{}))
                decision = engine.update(posed,loop,target_points)
                if engine.halted is None:
                    last_stamp,last_good = obs.stamp_ns,loop
                    processed += 1
                    floor_z = posed.world_from_base[2,3]
                source_age = (node.get_clock().now().nanoseconds-min(obs.color_stamp_ns,obs.depth_stamp_ns))/1e9
                elapsed = time.monotonic()-loop
                if source_age > .3 or elapsed > .3:
                    decision = Decision('PROCESSING_EXPIRED')
                metrics = {'processed':processed,'epoch':epoch,'source_age_sec':source_age,
                           'processing_sec':elapsed,'calibration_source':obs.calibration_source,
                           'upstream_commit':LOCKED_COMMIT,'perception_mode':perception_mode,
                           'vlm_enabled':False,**engine.state(),**target_metrics}
                report = node.emit(decision,obs.stamp_ns,metrics)
            except NoFreshFrame as exc:
                reason = str(exc)
                if engine.halted:
                    decision = Decision(engine.halted)
                elif (node.poses.reason or (last_good is not None and
                      (reason not in ('NO_NEW_OBSERVATION','NO_TIME_MATCHED_POSE','WAITING_FOR_POSE')
                       or loop-last_good > node.settings.frame_timeout))):
                    decision = engine.halt(node.poses.reason or reason)
                    node.publish_grid(None)
                else:
                    decision = Decision(reason)
                report = node.emit(decision,last_stamp,{'processed':processed,'epoch':epoch,
                    'perception_mode':perception_mode,'vlm_enabled':False,**engine.state()})
            except Exception as exc:
                report = node.emit(engine.halt('PROCESSING_ERROR'),last_stamp,
                                   {'error_type':type(exc).__name__,'error':str(exc),'processed':processed,'epoch':epoch,
                                    'perception_mode':perception_mode,'vlm_enabled':False,**engine.state()})
                node.publish_grid(None)
            node.publish_debug(engine,report,floor_z)
            if loop-last_save >= 1.:
                node.publish_grid(engine.grid,floor_z)
                recorder.save(report,engine.grid)
                node.get_logger().info(f"SHADOW {report.get('reason')} action={report.get('action')} frames={processed}; motion_output=false")
                last_save = loop
            time.sleep(max(0.,.1-(time.monotonic()-loop)))
    finally:
        final = {**Decision('EXITED').as_dict(),**engine.state(),'phase':'EXITED',
                 'initial_scan_skipped':engine.cfg.initial_scan_angle==0,
                 'source_stamp_ns':last_stamp,'processed':processed,
                 'epoch':epoch,'shadow':True,'motion_output':False,'camera':node.config['camera'],
                 'perception_mode':perception_mode,'vlm_enabled':False}
        if rclpy.ok():
            node.emit(Decision('EXITED'),last_stamp,final)
            node.debug_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        # Retain last decision separately, without claiming an exited process is active.
        final['last_decision'] = report
        recorder.save(final,engine.grid)
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
    return final


def main(args=None):
    parser=argparse.ArgumentParser(description='READ-ONLY random-frontier/FMM preview; NEVER drives the robot')
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--camera',choices=['astra_s','d455'])
    parser.add_argument('--duration',type=float,default=0.)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--perception',action='store_true',help='Enable local models; remains READ-ONLY')
    parser.add_argument('--skip-initial-scan',action='store_true',help='Stationary preview only; explicitly skip scan, not a completed scan')
    parser.add_argument('--target',default='chair')
    parser.add_argument('--device',choices=['cpu','cuda'],default='cuda')
    opts=parser.parse_args(args)
    if not np.isfinite(opts.duration) or opts.duration < 0: parser.error('duration must be >=0')
    config=load_configuration(opts.config,opts.camera)
    if opts.skip_initial_scan:
        config['map_config'] = replace(config['map_config'],initial_scan_angle=0.)
    engine=Explorer(config['map_config'],load_fmm(config['upstream_root']))
    perception=None
    if opts.perception:
        from .perception import LocalPerception
        perception=LocalPerception(Path(config['upstream_root']).parent.parent/'weights',opts.target,opts.device)
    rclpy.init()
    node=None
    try:
        node=PreviewNode(config)
        print('READ-ONLY: no motion output; no VLM; camera='+config['camera'],flush=True)
        if not config['extrinsics_confirmed']:
            print('PROVISIONAL camera installation: projection requires human verification.',flush=True)
        run(node,engine,opts.duration,opts.output,perception)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
