"""Isolated ROS domain, synthetic RGB-D/Odometry only. No device/base processes."""
from array import array
from pathlib import Path
import copy
import json
import threading
import time
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wheeltec_rgbd_interfaces.msg import RGBDFrame
from wheeltec_rgbd.node import latest_qos
from wheeltec_exploration.configuration import load_configuration
from wheeltec_exploration.core import Explorer
from wheeltec_exploration.offline import virtual_observation
from wheeltec_exploration.runtime import PreviewNode, run
from wheeltec_exploration.upstream import load_fmm


class Harness:
    def __init__(self):
        values={'control_enabled':'true','extrinsics_calibrated':'true',
            'odom_topic':'/exploration_test/odom','cmd_vel_topic':'/exploration_test/synthetic_output',
            'action_name':'/exploration_test/execute','stop_service':'/exploration_test/stop',
            'navigation_topic':'/exploration_test/navigation/cmd_vel',
            'navigation_mode_service':'/exploration_test/navigation/set_enabled',
            'status_topic':'/exploration_test/status'}
        args=['--ros-args']
        for k,v in values.items(): args.extend(['-p','wheeltec_motion_controller:'+k+':='+v])
        rclpy.init(args=args)
        path=Path(__file__).parents[1]/'config/astra_preview.yaml'
        self.cfg=load_configuration(path)
        self.cfg['odom_topic']='/exploration_test/odom'
        self.cfg['prefix']='/exploration_test/preview'
        self.cfg['body_from_base']=np.eye(4)
        self.cfg['base_from_camera']=virtual_observation().world_from_camera
        self.cfg['extrinsics_source']='synthetic_test'
        self.node=PreviewNode(self.cfg,topic='/exploration_test/rgbd',status_topic='/exploration_test/camera_status')
        self.source=Node('exploration_synthetic_source')
        self.rgb_pub=self.source.create_publisher(RGBDFrame,'/exploration_test/rgbd',latest_qos())
        self.odom_pub=self.source.create_publisher(Odometry,self.cfg['odom_topic'],latest_qos())
        self.reports=[]
        self.sub=self.source.create_subscription(String,self.cfg['prefix']+'/decision',lambda m:self.reports.append(json.loads(m.data)),1)
        self.reset=self.source.create_client(Trigger,self.cfg['prefix']+'/reset')
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.source)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True)
        self.thread.start()
        self.timer=self.source.create_timer(.05,self.send)
        self.publish=True
        self.jump=False

    def send(self):
        if not self.publish:return
        stamp=self.source.get_clock().now().to_msg()
        obs=virtual_observation().observation
        pose=Odometry()
        pose.header.stamp=stamp
        pose.header.frame_id='camera_init'
        pose.child_frame_id='body'
        pose.pose.pose.orientation.w=1.
        if self.jump: pose.pose.pose.position.x=3.
        self.odom_pub.publish(pose)
        msg=RGBDFrame()
        msg.header.stamp=stamp
        msg.header.frame_id='wheeltec_camera_color_optical_frame'
        msg.rectified=True
        msg.calibration_source='synthetic_test'
        for image in (msg.color,msg.depth):
            image.header=copy.deepcopy(msg.header)
            image.height,image.width=obs.depth_m.shape
        msg.color.encoding,msg.color.step='rgb8',64*3
        msg.color.data=array('B',obs.rgb.tobytes())
        msg.depth.encoding,msg.depth.step='32FC1',64*4
        msg.depth.data=array('B',obs.depth_m.astype('<f4').tobytes())
        for name in ('color_info','depth_info'):
            info=CameraInfo()
            info.header=copy.deepcopy(msg.header)
            info.height,info.width=obs.depth_m.shape
            info.k=obs.depth_info.k
            info.r=np.eye(3).ravel().tolist()
            info.d=[0.]*5
            info.distortion_model='plumb_bob'
            k=np.array(info.k).reshape(3,3)
            info.p=np.column_stack((k,np.zeros(3))).ravel().tolist()
            setattr(msg,name,info)
        self.rgb_pub.publish(msg)

    def wait(self,predicate,timeout=4.):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            if predicate():return
            time.sleep(.01)
        raise AssertionError('condition timed out')

    def start(self,output,duration=3.):
        self.engine=Explorer(self.cfg['map_config'],load_fmm(self.cfg['upstream_root']))
        self.result={}
        def worker(): self.result.update(run(self.node,self.engine,duration,output))
        self.worker=threading.Thread(target=worker)
        self.worker.start()
        self.wait(lambda:any(v['reason']=='PLANNED' for v in self.reports))

    def close(self):
        if hasattr(self,'worker'):self.worker.join(timeout=8.)
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.node.destroy_node()
        self.source.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def h():
    harness=Harness()
    try:yield harness
    finally:harness.close()


def test_shadow_stream_and_stale_stop_clear_map_no_motion_interfaces(h,tmp_path):
    h.start(tmp_path,2.2)
    pubs=h.source.get_publisher_names_and_types_by_node(h.node.get_name(),h.node.get_namespace())
    assert not any(name in ('/cmd_vel','/wheeltec_motion/navigation/cmd_vel') for name,_ in pubs)
    assert not h.source.get_publishers_info_by_topic('/cmd_vel')
    h.publish=False
    h.wait(lambda:any(v['reason'] in ('STALE','POSE_STALE') for v in h.reports))
    assert h.engine.grid is None
    assert all(v['shadow'] and not v['motion_output'] for v in h.reports)
    halted=[v for v in h.reports if v['reason'] in ('STALE','POSE_STALE')]
    assert all(v['linear_x']==v['angular_z']==0 for v in halted)
    h.worker.join(timeout=4.)
    assert not h.worker.is_alive()
    assert not (tmp_path/'map.npz').exists()
    assert json.loads((tmp_path/'report.json').read_text())['reason']=='EXITED'


def test_pose_jump_latches_until_explicit_reset(h,tmp_path):
    h.start(tmp_path,3.8)
    h.jump=True
    h.wait(lambda:any(v['reason']=='POSE_DISCONTINUITY' for v in h.reports))
    h.jump=False
    time.sleep(.2)
    assert h.engine.halted=='POSE_DISCONTINUITY' and h.engine.grid is None
    future=h.reset.call_async(Trigger.Request())
    h.wait(future.done)
    assert future.result().success
    h.wait(lambda:any(v['reason']=='PLANNED' and v['epoch']==1 for v in h.reports))
    assert h.engine.halted is None


def test_processing_deadline_previews_zero_without_replaying_old_frame(h,tmp_path):
    h.start(tmp_path,2.2)
    original=h.engine.update
    def delayed(*args,**kwargs):
        result=original(*args,**kwargs)
        time.sleep(.32)
        return result
    h.engine.update=delayed
    h.wait(lambda:any(v['reason']=='PROCESSING_EXPIRED' for v in h.reports))
    expired=[v for v in h.reports if v['reason']=='PROCESSING_EXPIRED']
    assert all(v['linear_x']==v['angular_z']==0 for v in expired)
    h.engine.update=original


def test_prepared_bridge_with_actual_controller_on_isolated_topics(h):
    from geometry_msgs.msg import Twist
    from wheeltec_motion_control.node import MotionNode
    from wheeltec_motion_control.navigation_client import NavigationClient
    from wheeltec_exploration.navigation_bridge import DecisionBridge
    from wheeltec_exploration.core import Decision
    commands=[]
    controller=MotionNode()
    h.executor.add_node(controller)
    sub=h.source.create_subscription(Twist,'/exploration_test/synthetic_output',lambda m:commands.append((m.linear.x,m.angular.z)),1)
    client=NavigationClient(prefix='/exploration_test',forward_speed=.1,turn_speed=.25)
    bridge=DecisionBridge(client,extrinsics_confirmed=True,calibration_verified=True)
    try:
        h.wait(lambda:controller.core.valid_pose_samples>=3)
        bridge.arm(1)
        for i in range(8):
            bridge.submit(Decision('PLANNED','forward',.1,0),client.get_clock().now().nanoseconds,1)
            time.sleep(.06)
        assert commands[-1][0]>0 and all(abs(v)<=.15 and abs(w)<=.35 for v,w in commands)
        bridge.submit(Decision('POSE_STALE'),client.get_clock().now().nanoseconds,1)
        h.wait(lambda:commands[-1]==(0.,0.))
        with pytest.raises(RuntimeError):bridge.submit(Decision('PLANNED','forward',.1,0),client.get_clock().now().nanoseconds,1)
        assert not h.source.get_publishers_info_by_topic('/cmd_vel')
    finally:
        bridge.stop()
        h.executor.remove_node(controller)
        controller.destroy_node()
        client.destroy_node()
        h.source.destroy_subscription(sub)
