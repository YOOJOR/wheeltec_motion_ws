from pathlib import Path
from wheeltec_rgbd.transport import camera_transport_environment
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('wheeltec_rgbd')) / 'config/astra_s.yaml')
    transport_env = camera_transport_environment(Path(config).with_name('fastdds_camera.xml'))
    calibration_dir = Path.home() / '.ros/camera_info'
    color_file, depth_file = calibration_dir / 'rgb_camera.yaml', calibration_dir / 'ir_camera.yaml'
    # Only the vendor's device driver; no vendor robot, mapping or navigation launch.
    driver = Node(package='astra_camera', executable='astra_camera_node',
                  name='astra_driver', namespace='/wheeltec_camera/raw', output='screen',
                  additional_env=transport_env,
                  condition=IfCondition(LaunchConfiguration('start_driver')),
                  parameters=[{
                      'camera_name': 'wheeltec_camera',
                      'camera_link_frame_id': 'wheeltec_camera_link',
                      'enable_color': True, 'enable_depth': True, 'enable_ir': False,
                      'color_width': 640, 'color_height': 480, 'color_fps': 30,
                      'depth_width': 640, 'depth_height': 480, 'depth_fps': 30,
                      'depth_registration': True, 'color_depth_synchronization': True,
                      'enable_point_cloud': False, 'enable_colored_point_cloud': False,
                      'publish_tf': True, 'tf_publish_rate': 10.0,
                      'enable_d2c_viewer': False, 'enable_publish_extrinsic': False,
                      'color_qos': 'SENSOR_DATA', 'depth_qos': 'SENSOR_DATA',
                      'color_camera_info_qos': 'SENSOR_DATA',
                      'depth_camera_info_qos': 'SENSOR_DATA',
                      'serial_number': ParameterValue(LaunchConfiguration('serial_number'), value_type=str),
                      'color_info_url': ParameterValue(LaunchConfiguration('color_info_url'), value_type=str),
                      'ir_info_url': ParameterValue(LaunchConfiguration('depth_info_url'), value_type=str),
                  }])
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('serial_number', default_value=''),
        DeclareLaunchArgument('color_info_url', default_value=color_file.as_uri() if color_file.is_file() else ''),
        DeclareLaunchArgument('depth_info_url', default_value=depth_file.as_uri() if depth_file.is_file() else ''),
        DeclareLaunchArgument('calibration_source', default_value='local_yaml_device_binding_unverified' if color_file.is_file() or depth_file.is_file() else 'driver_camera_info'),
        DeclareLaunchArgument('adapter_config', default_value=config),
        driver,
        Node(package='wheeltec_rgbd', executable='rgbd_adapter', name='rgbd_adapter',
             additional_env=transport_env,
             parameters=[LaunchConfiguration('adapter_config'), {
                 'calibration_source': ParameterValue(LaunchConfiguration('calibration_source'), value_type=str),
                 'color_calibration_url': ParameterValue(LaunchConfiguration('color_info_url'), value_type=str),
                 'depth_calibration_url': ParameterValue(LaunchConfiguration('depth_info_url'), value_type=str)
             }], output='screen'),
    ])
