from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stereo_depth_ros2_dir = get_package_share_directory('stereo_depth_ros2')
    map_manager_dir = get_package_share_directory('map_manager')

    stereo_param_path = os.path.join(stereo_depth_ros2_dir, 'cfg', 'stereo_depth_node_param.yaml')
    ov_config_path = os.path.join(stereo_depth_ros2_dir, 'config', 'openvins', 'estimator_config.yaml')
    map_param_path = os.path.join(map_manager_dir, 'cfg', 'map_param.yaml')
    rviz_config_path = os.path.join(map_manager_dir, 'rviz', 'map.rviz')

    # Stereo cameras: raw images for VIO, depth for mapping
    stereo_depth_node = Node(
        package='stereo_depth_ros2',
        executable='stereo_depth_node',
        name='stereo_depth_node',
        output='screen',
        parameters=[stereo_param_path]
    )

    # ICM-20948 IMU
    imu_node = Node(
        package='icm20948_ros2',
        executable='icm20948_node',
        name='icm20948_node',
        output='screen'
    )

    # OpenVINS stereo-inertial odometry
    # Remap /ov/odomimu to /unitree_go2/odom so map_manager can consume it
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
        ],
        remappings=[
            ('/ov/odomimu', '/unitree_go2/odom'),
        ]
    )

    # Occupancy mapping
    occupancy_map_node = Node(
        package='map_manager',
        executable='occupancy_map_node',
        name='map_manager_node',
        output='screen',
        parameters=[map_param_path]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

# VIO Watchdog Node
    vio_watchdog_node = Node(
        package='stereo_depth_ros2',
        executable='vio_watchdog.py',
        name='vio_watchdog',
        output='screen',
        parameters=[{
            'odom_topic': '/unitree_go2/odom',
            'status_topic': '/vio/status',
            'diagnostics_path': '/tmp/vio_diagnostics.log',
            'position_threshold_m': 50.0,
            'velocity_threshold_m_s': 10.0,
            'delta_threshold_m': 5.0,
            'window_size': 10,
            'log_max_lines': 2000,
        }]
    )

    return LaunchDescription([
        stereo_depth_node,
        imu_node,
        openvins_node,
        occupancy_map_node,
        vio_watchdog_node,
        rviz_node,
    ])
