"""Factory geometry composition, ground projection and isolated TF publication."""
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from wheeltec_rgbd.frame_tree import ground_point, installation_edges

CONFIG=Path(__file__).resolve().parents[4]/'conavgpt2_ws/development/Co-NavGPT2/wheeltec/config.yaml'


def test_factory_geometry_composition():
    edges=installation_edges(CONFIG)
    transforms={(p,c):m for p,c,m in edges}
    assert len({c for p,c,m in edges})==len(edges)
    base_camera=transforms['base_link','camera_link']
    footprint_base=np.linalg.inv(transforms['base_link','base_footprint'])
    np.testing.assert_allclose((footprint_base@base_camera)[:3,3],
                              [.16083451891873,.000745202407130424,.115704])
    np.testing.assert_allclose(transforms['body','base_link'][:3,3],[-.10718,-.02237,-.17788])
    yaw=np.eye(4);yaw[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    np.testing.assert_allclose((yaw@base_camera)[:3,3],[-.000745202407130424,.16083451891873,.089704])
    assert len(installation_edges(CONFIG,'d455'))==2


def test_ground_offset_rotates_with_body():
    base=np.eye(4);base[:3,:3]=Rotation.from_euler('xyz',[.2,-.3,.6]).as_matrix()
    base[:3,3]=[1.,2.,3.]
    ground=ground_point(base,.026)
    np.testing.assert_allclose(base[:3,:3].T@(ground[:3]-base[:3,3]),[0,0,-.026],atol=1e-14)
    assert not np.allclose(ground[:2],base[:2,3])


def test_tf_messages_match_configuration():
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster,Buffer,TransformListener
    import time
    rclpy.init()
    node=rclpy.create_node('synthetic_installation_tf_test')
    buffer=Buffer();listener=TransformListener(buffer,node)
    publisher=StaticTransformBroadcaster(node)
    expected=installation_edges(CONFIG)
    msgs=[]
    for parent,child,m in expected:
        msg=TransformStamped();msg.header.frame_id=parent;msg.child_frame_id=child
        msg.header.stamp=node.get_clock().now().to_msg()
        msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,m[:3,3])
        q=Rotation.from_matrix(m[:3,:3].copy()).as_quat()
        msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=map(float,q)
        msgs.append(msg)
    try:
        publisher.sendTransform(msgs)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and not buffer.can_transform('base_footprint','wheeltec_camera_color_optical_frame',rclpy.time.Time()):
            rclpy.spin_once(node,timeout_sec=.05)
        actual=buffer.lookup_transform('base_footprint','wheeltec_camera_color_optical_frame',rclpy.time.Time())
        assert abs(actual.transform.translation.z-.115704)<1e-10
    finally:
        node.destroy_node();rclpy.shutdown()
