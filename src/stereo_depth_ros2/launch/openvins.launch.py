from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stereo_depth_ros2_dir = get_package_share_directory('stereo_depth_ros2')

    stereo_param_path = os.path.join(stereo_depth_ros2_dir, 'cfg', 'stereo_depth_node_param.yaml')
    ov_config_path = os.path.join(stereo_depth_ros2_dir, 'config', 'openvins', 'estimator_config.yaml')

    # Stereo cameras: provides raw left/right images and IMU timestamp sync
    stereo_depth_node = Node(
        package='stereo_depth_ros2',
        executable='stereo_depth_node',
        name='stereo_depth_node',
        output='screen',
        parameters=[stereo_param_path]
    )

    # OpenVINS stereo-inertial odometry
    # Config path is passed as parameter; estimator_config.yaml contains relative paths
    # to cam_chain.yaml and imu_chain.yaml.
    openvins_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='ov_msckf',
        namespace='ov',
        output='screen',
        parameters=[
            {'verbosity': 'INFO'},
            {'use_stereo': True},
            {'max_cameras': 2},
            {'save_total_state': False},
            {'config_path': ov_config_path},
        ]
    )

    return LaunchDescription([
        stereo_depth_node,
        openvins_node,
    ])
