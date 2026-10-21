"""Standalone D455 driver; no image relay or robot commands."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_number', default_value=''),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='d455', namespace='/camera/d455/raw',
             parameters=[str(Path(__file__).resolve().parent.parent/'config/d455.yaml'),
                         {'serial_no': ParameterValue(LaunchConfiguration('serial_number'), value_type=str)}],
             output='screen')])
