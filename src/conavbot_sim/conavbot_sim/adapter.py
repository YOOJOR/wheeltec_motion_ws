"""Gazebo stamps mapped once to wall epoch; publish the deployed D455/LIO contract."""
import argparse
from collections import OrderedDict
from copy import deepcopy
import time
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class Adapter(Node):
    def __init__(self, config):
        super().__init__('conavbot_sim_adapter')
        self.config = config['adapter']; self.offset = None; self.clock_ns = None
        self.last_clock_wall = time.monotonic(); self.invalid = False
        self.clock_start = self.clock_window = None
        self.last_frame = -1; self.frames = OrderedDict()
        self.tf = TransformBroadcaster(self); self.static_tf = StaticTransformBroadcaster(self)
        self.odom = self.create_publisher(Odometry,'/Odometry',10)
        self.truth = self.create_publisher(Odometry,'/sim/ground_truth',10)
        self.emergency = self.create_publisher(Bool,'/sim/emergency_stop',10)
        self.stop = self.create_client(Trigger,'/wheeltec_motion/stop')
        self.cancel = self.create_client(Trigger,'/sim/cancel_goal')
        self.create_subscription(Clock,'/clock',self.clock,10)
        self.create_subscription(Odometry,'/sim/odometry',self.pose,10)
        prefix = '/camera/d455/raw/d455/'
        self.outputs = {key:self.create_publisher(kind,prefix+topic,10) for key,kind,topic in (
            ('rgb',Image,'color/image_raw'),('depth',Image,'aligned_depth_to_color/image_raw'),
            ('rgb_info',CameraInfo,'color/camera_info'),('depth_info',CameraInfo,'aligned_depth_to_color/camera_info'))}
        for key,kind,topic in [('rgb',Image,'image'),('depth',Image,'depth_image'),('info',CameraInfo,'camera_info')]:
            self.create_subscription(kind,'/sim/rgbd/'+topic,lambda msg,k=key:self.frame(k,msg),qos_profile_sensor_data)
        t = self.config['body_from_base_translation']; r = self.config['body_from_base_rpy']
        self.body_from_base = np.eye(4); self.body_from_base[:3,:3] = Rotation.from_euler('xyz',r).as_matrix()
        self.body_from_base[:3,3] = t
        fixed = TransformStamped(); fixed.header.frame_id='body'; fixed.child_frame_id='base_link'
        fixed.transform.translation.x, fixed.transform.translation.y, fixed.transform.translation.z = map(float,t)
        q = Rotation.from_euler('xyz',r).as_quat()
        fixed.transform.rotation.x,fixed.transform.rotation.y,fixed.transform.rotation.z,fixed.transform.rotation.w = map(float,q)
        self.static_tf.sendTransform(fixed)
        self.create_timer(.1,self.watch_clock)

    def fail(self, reason):
        if self.invalid: return
        self.invalid = True; self.frames.clear()
        self.get_logger().error(reason+'; simulation session latched invalid; restart session to reset epoch')
        self.emergency.publish(Bool(data=True))
        for client in (self.stop,self.cancel):
            if client.service_is_ready(): client.call_async(Trigger.Request())

    def clock(self, msg):
        stamp = ns(msg.clock)
        if self.clock_ns is not None and stamp < self.clock_ns:
            self.fail('SIM_CLOCK_RESET')
        if self.clock_ns != stamp: self.last_clock_wall = time.monotonic()
        self.clock_ns = stamp
        if self.offset is None:
            now = time.monotonic()
            if self.clock_start is None:
                self.clock_start = now
                self.clock_window = (now, stamp)
                self.get_logger().info('Waiting for rendering warmup and real-time clock before fixing epoch')
            elapsed = now-self.clock_window[0]
            if elapsed >= 1.0:
                rate = (stamp-self.clock_window[1])*1e-9/elapsed
                if now-self.clock_start >= 3.0 and .98 <= rate <= 1.02:
                    self.offset = time.time_ns()-stamp
                    self.get_logger().info(f'SIM_EPOCH_OFFSET_NS={self.offset}; initial real-time factor={rate:.3f}')
                self.clock_window = (now, stamp)

    def watch_clock(self):
        if self.offset is not None and time.monotonic()-self.last_clock_wall > self.config['clock_pause_timeout']:
            self.fail('SIM_CLOCK_PAUSED')

    def stamp(self, value):
        sec, nano = divmod(value+self.offset,1_000_000_000)
        return Time(sec=sec,nanosec=nano)

    def fresh(self, value):
        return self.offset is not None and not self.invalid and self.clock_ns is not None and -100_000_000 <= self.clock_ns-value <= self.config['max_frame_age']*1e9

    def pose(self, msg):
        source = ns(msg.header.stamp)
        if not self.fresh(source): return
        truth = deepcopy(msg); truth.header.stamp=self.stamp(source)
        truth.header.frame_id='camera_init'; truth.child_frame_id='base_link'; self.truth.publish(truth)
        p = msg.pose.pose.position; q=msg.pose.pose.orientation
        world_base=np.eye(4); world_base[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        world_base[:3,3]=[p.x,p.y,p.z]
        world_body = world_base @ np.linalg.inv(self.body_from_base)
        body=deepcopy(truth); body.child_frame_id='body'
        p=body.pose.pose.position; p.x,p.y,p.z=map(float,world_body[:3,3])
        q=body.pose.pose.orientation; q.x,q.y,q.z,q.w=map(float,Rotation.from_matrix(world_body[:3,:3]).as_quat())
        # Body twist is not used by motion. Clear base-frame values at this boundary.
        body.twist=type(body.twist)(); self.odom.publish(body)
        tf=TransformStamped(); tf.header=deepcopy(body.header); tf.child_frame_id='body'
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=p.x,p.y,p.z
        tf.transform.rotation=deepcopy(q); self.tf.sendTransform(tf)

    def frame(self, key, msg):
        source=ns(msg.header.stamp)
        if source <= self.last_frame or not self.fresh(source): return
        self.frames.setdefault(source,{})[key]=msg
        while len(self.frames)>3: self.frames.popitem(last=False)
        candidates=[stamp for stamp,parts in self.frames.items() if len(parts)==3]
        if not candidates: return
        source=max(candidates); parts=self.frames[source]
        rgb=deepcopy(parts['rgb']); depth=deepcopy(parts['depth']); info=deepcopy(parts['info'])
        if rgb.encoding!='rgb8' or depth.encoding!='32FC1':
            self.fail('SIM_RGBD_ENCODING_MISMATCH'); return
        if (rgb.height,rgb.width)!=(depth.height,depth.width) or (info.height,info.width)!=(rgb.height,rgb.width):
            self.fail('SIM_RGBD_SIZE_MISMATCH'); return
        dtype=np.dtype('>f4' if depth.is_bigendian else '<f4')
        values=np.ndarray((depth.height,depth.width),dtype=dtype,buffer=bytes(depth.data),strides=(depth.step,4))
        valid=np.isfinite(values)&(values>0)&(values<65.535)
        mm=np.zeros(values.shape,dtype='<u2'); mm[valid]=np.rint(values[valid]*1000).astype('<u2')
        depth.encoding='16UC1'; depth.is_bigendian=0; depth.step=depth.width*2; depth.data=mm.tobytes()
        for message in (rgb,depth,info):
            message.header.stamp=self.stamp(source); message.header.frame_id='d455_color_optical_frame'
        self.outputs['rgb'].publish(rgb); self.outputs['depth'].publish(depth)
        self.outputs['rgb_info'].publish(info); self.outputs['depth_info'].publish(info)
        self.last_frame=source
        for stamp in list(self.frames):
            if stamp<=source: del self.frames[stamp]


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--config',required=True)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args); node=Adapter(yaml.safe_load(open(args.config)))
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
