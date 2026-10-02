"""D455 camera-only acquisition into the shared RGB-D contract."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('wheeltec_rgbd')) / 'config/d455.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('adapter_config', default_value=config),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             name='wheeltec_d455', namespace='/wheeltec_camera/d455/raw', output='screen',
             condition=IfCondition(LaunchConfiguration('start_driver')),
             parameters=[{
                 'camera_name': 'wheeltec_d455', 'device_type': 'd455',
                 'serial_no': ParameterValue(LaunchConfiguration('serial_number'), value_type=str),
                 'enable_color': True, 'enable_depth': True,
                 'rgb_camera.color_profile': '640,480,15',
                 'depth_module.depth_profile': '640,480,15',
                 'rgb_camera.color_format': 'RGB8', 'depth_module.depth_format': 'Z16',
                 'enable_sync': True, 'align_depth.enable': True,
                 'enable_gyro': False, 'enable_accel': False,
                 'enable_infra1': False, 'enable_infra2': False,
                 'pointcloud.enable': False, 'publish_tf': True,
                 'wait_for_device_timeout': 10.0,
             }]),
        Node(package='wheeltec_rgbd', executable='rgbd_adapter', name='rgbd_adapter', namespace='/wheeltec_camera/d455',
             parameters=[LaunchConfiguration('adapter_config'), {
                 'calibration_source': 'realsense_device_camera_info',
                 'color_calibration_url': '', 'depth_calibration_url': '',
                 'hardware_id': 'realsense_d455',
                 'timestamp_origin': 'realsense_driver_ros_timestamp',
             }], output='screen'),
    ])
