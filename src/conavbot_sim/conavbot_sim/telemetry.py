"""Display driver wheel dynamics without producing motion requests."""
import math

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from ros_gz_interfaces.msg import Float32Array
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker


class MotorTelemetry(Node):
    def __init__(self):
        super().__init__('sim_motor_telemetry')
        self.target = self.create_publisher(JointState, '/sim/wheel_target', 1)
        self.measured = self.create_publisher(JointState, '/sim/wheel_measured', 1)
        self.delayed = self.create_publisher(Twist, '/sim/delayed_cmd_vel', 1)
        self.markers = self.create_publisher(Marker, '/sim/motor_markers', 1)
        self.tracks = self.create_publisher(Path, '/sim/trajectory', 1)
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'camera_init'
        self.last_track = 0
        self.pose = None
        self.actual_vx = self.actual_wz = 0.
        self.last_driver = None
        self.driver_time = None
        self.state = None
        self.create_subscription(Float32Array, '/sim/motor_state', self.on_motor, 1)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, 1)
        self.create_timer(.2, self.publish_marker)

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        self.actual_vx = msg.twist.twist.linear.x
        self.actual_wz = msg.twist.twist.angular.z
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if stamp-self.last_track >= 100_000_000:
            self.last_track = stamp
            self.trajectory.header.stamp = msg.header.stamp
            self.trajectory.poses.append(PoseStamped(header=msg.header, pose=msg.pose.pose))
            self.trajectory.poses = self.trajectory.poses[-4000:]
            self.tracks.publish(self.trajectory)

    def on_motor(self, msg):
        values = list(msg.data)
        if len(values) != 20 or not all(math.isfinite(v) for v in values):
            self.get_logger().error('motor_state must contain 20 finite values')
            return
        self.state = values
        self.last_driver = self.get_clock().now()
        self.driver_time = values[0]
        names = ['lf_wheel_joint', 'lb_wheel_joint', 'rb_wheel_joint', 'rf_wheel_joint']
        target = JointState()
        target.header.stamp = self.last_driver.to_msg()
        target.name = names
        target.velocity = [float(v) for v in values[2:6]]
        measured = JointState()
        measured.header = target.header
        measured.name = names
        measured.velocity = [float(v) for v in values[6:10]]
        measured.effort = [float(v) for v in values[10:14]]
        self.target.publish(target)
        self.measured.publish(measured)
        delayed = Twist()
        delayed.linear.x, delayed.linear.y, delayed.angular.z = [float(v) for v in values[17:20]]
        self.delayed.publish(delayed)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'camera_init'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'motor_telemetry'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.lifetime.nanosec = 500_000_000
        marker.pose.orientation.w = 1.
        marker.pose.position.z = 2.5
        if self.pose is not None:
            marker.pose.position.x = self.pose.position.x
            marker.pose.position.y = self.pose.position.y
        marker.scale.z = .14
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.
        if self.state is None:
            marker.text = 'Motor telemetry: waiting for driver'
        else:
            s = self.state
            age = (self.get_clock().now()-self.last_driver).nanoseconds*1e-9
            marker.text = (
                'LF LB RB RF [rad/s] | driver t=%.2f s%s\n'
                'target:  %+.2f %+.2f %+.2f %+.2f\n'
                'actual:  %+.2f %+.2f %+.2f %+.2f\n'
                'torque [Nm]: %+.2f %+.2f %+.2f %+.2f\n'
                'received vx/vy/wz: %+.2f %+.2f %+.2f\n'
                'delayed  vx/vy/wz: %+.2f %+.2f %+.2f\n'
                'chassis vx= %+.2f m/s, wz= %+.2f rad/s | latch=%d'
            ) % (s[0], ' STALE' if age > .5 else '', *s[2:14], *s[14:20],
                 self.actual_vx, self.actual_wz, int(s[1]))
        self.markers.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MotorTelemetry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
