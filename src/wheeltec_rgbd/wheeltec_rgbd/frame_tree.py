"""Read-only installation TF; never opens devices or creates motion endpoints."""
import argparse
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .posed import rigid, transform


def ground_point(base, height):
    if not np.isfinite(height) or height < 0:
        raise ValueError('INVALID_BASE_HEIGHT')
    return rigid(base) @ np.array([0., 0., -height, 1.])


def installation_edges(config, camera='astra_s'):
    c = yaml.safe_load(Path(config).read_text())
    if c.get('base_frame') != 'base_link':
        raise ValueError('BASE_FRAME_MISMATCH')
    m = yaml.safe_load(Path(c['motion_config']).read_text())['wheeltec_motion_controller']['ros__parameters']
    if m['navigation_frame'] != 'base_link':
        raise ValueError('MOTION_FRAME_MISMATCH')
    body_base = transform(m['body_from_base_translation'],
                          Rotation.from_euler('xyz', m['body_from_base_rpy']).as_quat())
    footprint = np.eye(4)
    footprint[:3, 3] = ground_point(np.eye(4), c['base_height'])[:3]
    edges = [('body', 'base_link', body_base), ('base_link', 'base_footprint', footprint)]
    if camera == 'astra_s':
        optical = np.eye(4)
        optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        base_optical = rigid(c['cameras'][camera]['base_from_camera'])
        edges += [('base_link', 'camera_link', base_optical @ np.linalg.inv(optical)),
                  ('camera_link', 'wheeltec_camera_link', np.eye(4)),
                  ('wheeltec_camera_link', 'wheeltec_camera_color_optical_frame', optical)]
    elif camera != 'd455':
        raise ValueError('UNSUPPORTED_CAMERA')
    # D455 keeps its driver-internal TF but has no asserted robot installation.
    return edges


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--camera', choices=['astra_s', 'd455'], default='astra_s')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    edges = installation_edges(args.config, args.camera)
    if args.check:
        print('Installation TF validated: ' + ', '.join(a+' -> '+b for a,b,_ in edges))
        return
    import fcntl
    import os
    lock = open(f"/tmp/wheeltec-installation-tf-{os.getuid()}.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("Installation TF publisher already active; reuse the existing visualization session")
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    rclpy.init()
    node = rclpy.create_node('wheeltec_installation_tf')
    broadcaster = StaticTransformBroadcaster(node)
    messages = []
    for parent, child, matrix in edges:
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, matrix[:3,3])
        q = Rotation.from_matrix(matrix[:3,:3].copy()).as_quat()
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float,q)
        messages.append(msg)
    broadcaster.sendTransform(messages)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
