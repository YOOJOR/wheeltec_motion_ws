"""Separate-process VGA transport regression; synthetic data and isolated ROS domain."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import pytest
import rclpy
from rclpy.node import Node
from wheeltec_rgbd.node import latest_qos
from wheeltec_rgbd_interfaces.msg import RGBDFrame


PUBLISHER = '''
import json,time
from array import array
import rclpy
from rclpy.node import Node
from wheeltec_rgbd.node import latest_qos
from wheeltec_rgbd_interfaces.msg import RGBDFrame
rclpy.init();node=Node('synthetic_large_rgbd_publisher')
pub=node.create_publisher(RGBDFrame,'/rgbd_transport_test/frame',latest_qos())
deadline=time.monotonic()+8
while pub.get_subscription_count()==0 and time.monotonic()<deadline:
 rclpy.spin_once(node,timeout_sec=.02)
if pub.get_subscription_count()==0:raise RuntimeError('no subscriber')
msg=RGBDFrame();msg.color.width=msg.depth.width=640;msg.color.height=msg.depth.height=480
msg.color.encoding='rgb8';msg.color.step=640*3;msg.color.data=array('B',[42])*(640*480*3)
msg.depth.encoding='32FC1';msg.depth.step=640*4;msg.depth.data=array('B',[0,0,128,63])*(640*480)
start=time.monotonic()
for i in range(120):
 msg.sequence=i+1;msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg)
 time.sleep(max(0,start+(i+1)/30-time.monotonic()))
time.sleep(.2)
print(json.dumps({'sent':120}),flush=True)
node.destroy_node();rclpy.shutdown()
'''


def test_configured_transport_delivers_continuous_vga_between_processes():
    if os.environ.get('ROS_DOMAIN_ID','0') == '0':
        pytest.skip('run with isolated ROS_DOMAIN_ID')
    profile=Path(__file__).parents[1]/'config/fastdds_camera.xml'
    env=dict(os.environ,FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    env.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
    rclpy.init()
    node=Node('synthetic_large_rgbd_subscriber')
    received=[]
    def receive(msg):
        received.append((time.monotonic(),msg.sequence,
            (node.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9,
            len(msg.color.data),len(msg.depth.data),msg.color.data[-1],list(msg.depth.data[-4:])))
    node.create_subscription(RGBDFrame,'/rgbd_transport_test/frame',receive,latest_qos(4))
    child=subprocess.Popen([sys.executable,'-c',PUBLISHER],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        deadline=time.monotonic()+18
        while child.poll() is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
        stdout,stderr=child.communicate(timeout=2)
        assert child.returncode == 0,stderr
        assert json.loads(stdout.strip().splitlines()[-1])['sent']==120
        assert len(received)>=114, (len(received),stderr)
        assert all(b[1]>a[1] for a,b in zip(received,received[1:]))
        assert max(b[0]-a[0] for a,b in zip(received,received[1:]))<.3
        assert max(r[2] for r in received)<.5
        assert all(r[3:]==(640*480*3,640*480*4,42,[0,0,128,63]) for r in received)
    finally:
        if child.poll() is None:
            child.terminate();child.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()
