"""Small action client; all target angles are radians."""

import argparse
import json
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from wheeltec_motion_interfaces.action import ExecuteMotion


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('motion', choices=['move', 'rotate', 'stop'])
    parser.add_argument('target', type=float, nargs='?', default=0.0)
    parser.add_argument('--max-speed', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=0.0, help='action timeout; 0 uses YAML')
    parser.add_argument('--server-wait', type=float, default=5.0)
    parser.add_argument('--action', default='/wheeltec_motion/execute')
    parser.add_argument('--stop-service', default='/wheeltec_motion/stop')
    options = parser.parse_args(remove_ros_args(args=sys.argv if args is None else args)[1:])
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('wheeltec_motion_client')
    handle = None
    exit_code = 1
    try:
        if options.motion == 'stop':
            client = node.create_client(Trigger, options.stop_service)
            if not client.wait_for_service(timeout_sec=options.server_wait):
                raise RuntimeError('stop service unavailable')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=options.server_wait)
            if not future.done():
                raise RuntimeError('stop response timed out')
            response = future.result()
            print(response.message)
            exit_code = 0 if response.success else 1
        else:
            client = ActionClient(node, ExecuteMotion, options.action)
            if not client.wait_for_server(timeout_sec=options.server_wait):
                raise RuntimeError('action server unavailable')
            goal = ExecuteMotion.Goal()
            goal.motion_type = goal.MOVE_LINEAR if options.motion == 'move' else goal.ROTATE
            goal.target, goal.max_speed, goal.timeout_sec = options.target, options.max_speed, options.timeout

            def feedback(msg):
                f = msg.feedback
                print(f'{f.phase}: remaining={f.remaining:.4f}, cross={f.cross_track_error:.4f}, '
                      f'elapsed={f.elapsed_sec:.1f}s', flush=True)

            future = client.send_goal_async(goal, feedback_callback=feedback)
            rclpy.spin_until_future_complete(node, future, timeout_sec=options.server_wait)
            if not future.done():
                raise RuntimeError('goal acceptance timed out; inspect server or use stop service')
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('goal rejected; see controller log')
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            result = result_future.result().result
            print(json.dumps({
                'code': result.code, 'message': result.message,
                'pose': [result.final_x, result.final_y, result.final_yaw],
                'remaining': result.remaining, 'elapsed_sec': result.elapsed_sec}, ensure_ascii=False))
            exit_code = 0 if result.code == result.SUCCEEDED else 1
    except KeyboardInterrupt:
        # A disconnected client is not itself a stop signal. Prefer explicit stop
        # in another terminal if this process was killed or its ROS context ended.
        if handle is not None and handle.accepted and rclpy.ok():
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=options.server_wait)
        exit_code = 130
    except Exception as exc:
        print(str(exc), file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
