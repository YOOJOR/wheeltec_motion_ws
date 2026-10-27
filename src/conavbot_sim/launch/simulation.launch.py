"""Start the physical world with no navigation task or nonzero command."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('conavbot_sim'))
    generated = Path('/sim/runtime/generated')
    processes = [
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', str(generated / 'room.sdf')],
                       output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config/bridge.yaml')}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (generated / 'robot.urdf').read_text()}]),
        Node(package='wheeltec_motion_control', executable='motion_controller',
             parameters=[str(generated / 'motion.yaml'), {'use_sim_time': False}], output='screen'),
        ExecuteProcess(cmd=['python3', '-m', 'conavbot_sim.adapter', '--config',
                            str(share / 'config/simulation.yaml')], output='screen'),
        ExecuteProcess(cmd=['python3', '-m', 'conavbot_sim.telemetry'], output='screen'),
        ExecuteProcess(cmd=['python3', '-m', 'conavbot_sim.exploration_status'], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'config/debug.rviz')], output='screen'),
    ]
    exits = [RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(reason='A simulation component exited'))]))
             for process in processes]
    return LaunchDescription(exits + processes)
