#!/usr/bin/env python3
"""Read-only bounded recording. Never enables navigation or publishes velocity."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from wheeltec_motion_interfaces.msg import MotionStatus
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'conavgpt2_ws/development/Co-NavGPT2'))
from conavbot.geometry import transform
from conavbot.perception import latest_qos
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration',type=float,default=60.)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sensor-timing',action='store_true',help='Also observe D455 color/depth header arrival times; never store image pixels')
    args=p.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=120:p.error('duration must be (0,120]')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    config=Path(__file__).parents[1]/'src/wheeltec_motion_control/config/motion_control.yaml'
    offset=np.array(yaml.safe_load(config.read_text())['wheeltec_motion_controller']['ros__parameters']['body_from_base_translation'])
    rclpy.init();node=Node('conav_acceptance_recorder')
    commands=[];poses=[];decisions=[];states=[];requests=[];sensor_timing=[]
    def sensor(kind,msg):
        now=node.get_clock().now().nanoseconds
        stamp=lambda h:h.stamp.sec*10**9+h.stamp.nanosec
        row={'kind':kind,'received_ros_ns':now,'received_monotonic':time.monotonic(),
             'source_stamp_ns':stamp(msg.header),'header_age_sec':(now-stamp(msg.header))/1e9}
        sensor_timing.append(row)
    if args.sensor_timing:
        node.create_subscription(Image,'/camera/d455/raw/d455/color/image_raw',lambda m:sensor('raw_color',m),latest_qos())
        node.create_subscription(Image,'/camera/d455/raw/d455/aligned_depth_to_color/image_raw',lambda m:sensor('raw_depth',m),latest_qos())
    def odometry(m):
        xyz,q=m.pose.pose.position,m.pose.pose.orientation
        matrix=transform([xyz.x,xyz.y,xyz.z],[q.x,q.y,q.z,q.w])
        base=matrix[:3,3]+matrix[:3,:3] @ offset
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        poses.append((time.monotonic(),base.tolist(),math.atan2(matrix[1,0],matrix[0,0]),(node.get_clock().now().nanoseconds-stamp)/1e9))
    node.create_subscription(Twist,'/cmd_vel',lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),1)
    node.create_subscription(Odometry,'/Odometry',odometry,latest_qos())
    node.create_subscription(String,'/conavbot/decision',lambda m:decisions.append(json.loads(m.data)),1)
    node.create_subscription(MotionStatus,'/wheeltec_motion/status',lambda m:states.append({'mode':m.mode,'phase':m.phase,'reason':m.reason,'request_age_sec':m.navigation_request_age_sec}),1)
    def navigation(m):
        stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        requests.append((time.monotonic(),stamp,(node.get_clock().now().nanoseconds-stamp)/1e9,m.twist.linear.x,m.twist.angular.z))
    node.create_subscription(TwistStamped,'/wheeltec_motion/navigation/cmd_vel',navigation,1)
    start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<args.duration:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        fresh=[v for v in decisions if 'source_age_sec' in v]
        unique={v['source_stamp_ns']:v for v in fresh}
        yaw=np.unwrap([v[2] for v in poses]) if poses else []
        result={'read_only':True,'duration_sec':time.monotonic()-start,
            'cmd_vel_publishers':[v.node_name for v in node.get_publishers_info_by_topic('/cmd_vel')],
            'commands_received':len(commands),'nonzero_commands':sum(abs(v)+abs(w)>1e-8 for _,v,w in commands),
            'max_abs_linear_x':max((abs(v) for _,v,w in commands),default=0.),
            'max_abs_angular_z':max((abs(w) for _,v,w in commands),default=0.),
            'last_command':commands[-1][1:] if commands else None,
            'last_status':states[-1] if states else None,
            'status_reasons':dict(Counter(v['reason'] for v in states)),
            'decision_reasons':dict(Counter(v['reason'] for v in decisions)),
            'decision_actions':dict(Counter(v.get('action') for v in unique.values())),
            'unique_observations':len(unique),
            'navigation_requests_received':len(requests),
            'navigation_request_age_max_sec':max((v[2] for v in requests),default=None),
            'source_age_max_sec':max((v['source_age_sec'] for v in fresh),default=None),
            'processing_max_sec':max((v['processing_sec'] for v in fresh),default=None),
            'odometry_age_max_sec':max((v[3] for v in poses),default=None),
            'base_displacement_m':float(np.linalg.norm(np.array(poses[-1][1])-poses[0][1])) if poses else None,
            'base_path_length_m':float(np.linalg.norm(np.diff([v[1] for v in poses],axis=0),axis=1).sum()) if poses else None,
            'yaw_change_rad':float(yaw[-1]-yaw[0]) if len(yaw) else None,
            'yaw_total_abs_rad':float(np.abs(np.diff(yaw)).sum()) if len(yaw) else None,
            'physical_stop_measured':False,
            'sensor_timing_enabled':args.sensor_timing,
            'sensor_timing':{kind:{'count':len(rows),'header_age_max_sec':max(r['header_age_sec'] for r in rows),
                'header_age_p95_sec':float(np.percentile([r['header_age_sec'] for r in rows],95))}
                for kind in ('raw_color','raw_depth') if (rows:=[r for r in sensor_timing if r['kind']==kind])}}
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        args.output.with_suffix('.samples.json').write_text(json.dumps({'commands':commands,'poses':poses,'decisions':decisions,'states':states,'requests':requests,'sensor_timing':sensor_timing})+'\n')
        print(json.dumps(result,indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
