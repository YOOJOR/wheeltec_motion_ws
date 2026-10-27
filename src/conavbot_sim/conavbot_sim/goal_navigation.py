"""RViz goals on the generated Gazebo map; motion remains in MotionNode."""
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker
from wheeltec_motion_interfaces.msg import MotionStatus, NavigationPath
from utils.fmm_planner import FMMPlanner


class GoalNavigation(Node):
    def __init__(self):
        super().__init__('sim_goal_navigation')
        for name, value in [('map_file', ''), ('footprint_radius', .286),
                            ('forward_speed', .10), ('turn_speed', .25)]:
            self.declare_parameter(name, value)
        self.goal = self.pose = self.status = None
        self.last_odom = self.used_odom = None
        self.active = self.pending = False
        self.stop_pending = False
        self.enable_stamp = None
        self.generation = 0
        self.text = 'READY: click 2D Goal Pose'
        self.mode = self.create_client(SetBool, '/wheeltec_motion/navigation/set_enabled')
        self.stop = self.create_client(Trigger, '/wheeltec_motion/stop')
        self.requests = self.create_publisher(NavigationPath, '/wheeltec_motion/navigation/path', 1)
        self.paths = self.create_publisher(NavPath, '/sim/planned_path', 1)
        self.messages = self.create_publisher(String, '/sim/navigation_status', 1)
        self.markers = self.create_publisher(Marker, '/sim/status_markers', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.maps = self.create_publisher(OccupancyGrid, '/sim/map', qos)
        self.load_map()
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 1)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_odom, 1)
        self.create_subscription(MotionStatus, '/wheeltec_motion/status', self.on_status, 1)
        self.create_service(Trigger, '/sim/cancel_goal', self.cancel)
        self.create_timer(.1, self.tick)
        self.create_timer(.5, self.publish_status)

    def load_map(self):
        filename = Path(self.get_parameter('map_file').value)
        data = yaml.safe_load(filename.read_text())
        self.resolution = float(data['resolution'])
        self.origin = data['origin']
        if abs(self.resolution-.05) > 1e-9 or abs(self.origin[2]) > 1e-9:
            raise ValueError('simulation map requires 0.05 m resolution and zero origin yaw')
        image = cv2.imread(str(filename.parent / data['image']), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('cannot read simulation map image')
        probability = image.astype(float)/255
        if not data.get('negate', 0):
            probability = 1-probability
        occupied = np.flipud(probability >= data.get('free_thresh', .196))
        radius = self.get_parameter('footprint_radius').value + .05
        cells = math.ceil(radius/self.resolution)
        yy, xx = np.mgrid[-cells:cells+1, -cells:cells+1]
        kernel = ((xx*xx+yy*yy)*self.resolution**2 <= radius**2).astype('uint8')
        blocked = cv2.dilate(occupied.astype('uint8'), kernel, borderType=cv2.BORDER_CONSTANT,
                             borderValue=1)
        self.free = blocked == 0
        self.planner = FMMPlanner(self.free.astype('uint8'))
        grid = OccupancyGrid()
        grid.header.frame_id = 'camera_init'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.resolution
        grid.info.height, grid.info.width = occupied.shape
        grid.info.origin.position.x, grid.info.origin.position.y = self.origin[:2]
        grid.info.origin.orientation.w = 1.
        grid.data = (occupied.astype('int8')*100).ravel().tolist()
        self.maps.publish(grid)

    def cell(self, x, y):
        return (math.floor((y-self.origin[1])/self.resolution),
                math.floor((x-self.origin[0])/self.resolution))

    def valid(self, cell):
        r, c = cell
        return 0 <= r < self.free.shape[0] and 0 <= c < self.free.shape[1] and self.free[r, c]

    def on_status(self, msg):
        self.status = msg

    def halt(self, reason):
        self.generation += 1
        self.goal = None
        self.active = self.pending = False
        self.text = reason
        self.paths.publish(self.path_message([], self.get_clock().now().to_msg()))
        if self.stop.service_is_ready():
            self.stop_pending = True
            self.stop.call_async(Trigger.Request()).add_done_callback(self.stopped)
        else:
            self.text += '; stop service unavailable'

    def stopped(self, future):
        response = future.result()
        self.stop_pending = False
        if not response.success:
            self.goal = None
            self.text += '; stop rejected: '+response.message

    def cancel(self, request, response):
        self.halt('CANCELLED: waiting for next goal')
        response.success = True
        response.message = 'goal cleared; stop requested'
        return response

    def on_goal(self, msg):
        self.halt('planning new goal')
        if not self.stop.service_is_ready():
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        if msg.header.frame_id != 'camera_init' or not all(math.isfinite(v) for v in (x, y)):
            self.text = 'REJECTED: goal must be finite camera_init coordinates'
            return
        cell = self.cell(x, y)
        if not self.valid(cell):
            self.text = 'REJECTED: goal outside safe free space'
            return
        self.planner.set_goal(cell)
        self.goal = (x, y)
        self.used_odom = self.last_odom
        self.text = 'WAITING: fresh odometry and motion controller'

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        self.last_odom = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def path_message(self, points, stamp):
        path = NavPath()
        path.header.frame_id, path.header.stamp = 'camera_init', stamp
        for x, y in points:
            point = PoseStamped()
            point.header = path.header
            point.pose.position.x, point.pose.position.y = float(x), float(y)
            point.pose.orientation.w = 1.
            path.poses.append(point)
        return path

    def descend(self, start):
        goal = self.cell(*self.goal)
        cell = start
        points = [(self.pose.position.x, self.pose.position.y)]
        distance = 0.
        while cell != goal:
            r, c = cell
            candidates = []
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nxt = r+dr, c+dc
                    if (dr == dc == 0 or not self.valid(nxt) or
                        (dr and dc and (not self.valid((r+dr, c)) or not self.valid((r, c+dc))))):
                        continue
                    if self.planner.fmm_dist[nxt] < self.planner.fmm_dist[cell]-1e-8:
                        candidates.append(nxt)
            if not candidates:
                raise ValueError('goal unreachable from current safe cell')
            cell = min(candidates, key=lambda p: self.planner.fmm_dist[p])
            x = self.origin[0]+(cell[1]+.5)*self.resolution
            y = self.origin[1]+(cell[0]+.5)*self.resolution
            distance += math.hypot(x-points[-1][0], y-points[-1][1])
            points.append((x, y))
            if distance >= 1.:
                return points
        points.append(self.goal)
        return points

    def enabled(self, future, generation):
        if generation != self.generation:
            return
        self.pending = False
        response = future.result()
        if not response.success:
            self.halt('FAILED: enable rejected: '+response.message)
            return
        self.active = True
        self.enable_stamp = self.get_clock().now().nanoseconds
        self.used_odom = self.last_odom
        self.text = 'WAITING: fresh odometry after enable'

    def tick(self):
        if self.goal is None or self.pose is None or self.stop_pending:
            return
        now = self.get_clock().now()
        if self.last_odom is None or (now.nanoseconds-self.last_odom)*1e-9 > 3.0:
            self.halt('STOPPED: odometry stale')
            return
        if self.last_odom == self.used_odom:
            return
        self.used_odom = self.last_odom
        stamp = now.to_msg()  # This decision starts with a previously unused odometry sample.
        start = self.cell(self.pose.position.x, self.pose.position.y)
        if not self.valid(start):
            self.halt('STOPPED: robot outside safe free space')
            return
        if math.hypot(self.goal[0]-self.pose.position.x, self.goal[1]-self.pose.position.y) <= .03:
            self.halt('REACHED: position error <= 0.03 m; waiting for next goal')
            return
        try:
            points = self.descend(start)
        except ValueError as error:
            self.halt('REJECTED: '+str(error))
            return
        if self.status is None or (now.nanoseconds-(self.status.stamp.sec*10**9+self.status.stamp.nanosec))*1e-9 > .6:
            self.halt('STOPPED: motion status stale')
            return
        if not self.active:
            if not self.pending and self.mode.service_is_ready() and self.status.pose_ready:
                request = SetBool.Request()
                request.data = True
                self.pending = True
                generation = self.generation
                self.mode.call_async(request).add_done_callback(lambda f: self.enabled(f, generation))
            return
        status_stamp = self.status.stamp.sec*10**9+self.status.stamp.nanosec
        if status_stamp <= self.enable_stamp:
            return
        if self.status.mode != MotionStatus.NAVIGATION:
            self.halt('STOPPED: controller left navigation mode')
            return
        path = self.path_message(points, stamp)
        request = NavigationPath()
        request.path = path
        request.max_linear_speed = self.get_parameter('forward_speed').value
        request.max_angular_speed = self.get_parameter('turn_speed').value
        self.requests.publish(request)
        self.paths.publish(path)
        self.text = 'TRACKING: FMM horizon %.2f m' % sum(math.dist(a,b) for a,b in zip(points, points[1:]))

    def publish_status(self):
        self.messages.publish(String(data=self.text))
        marker = Marker()
        marker.header.frame_id = 'camera_init'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'navigation_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.lifetime.nanosec = 800_000_000
        marker.pose.position.z = .65
        if self.pose is not None:
            marker.pose.position.x = self.pose.position.x
            marker.pose.position.y = self.pose.position.y
        marker.pose.orientation.w = 1.
        marker.scale.z = .18
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.
        marker.text = self.text
        self.markers.publish(marker)
        marker.ns = 'navigation_goal'
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = .12
        marker.color.r, marker.color.g, marker.color.b = 1., .4, 0.
        marker.action = Marker.ADD if self.goal is not None else Marker.DELETE
        if self.goal is not None:
            marker.pose.position.x, marker.pose.position.y = self.goal
            marker.pose.position.z = .1
        self.markers.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.halt('STOPPED: navigation shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
