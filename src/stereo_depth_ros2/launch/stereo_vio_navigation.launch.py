from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stereo_depth_ros2_dir = get_package_share_directory('stereo_depth_ros2')
    map_manager_dir = get_package_share_directory('map_manager')
    onboard_detector_dir = get_package_share_directory('onboard_detector')
    navigation_runner_dir = get_package_share_directory('navigation_runner')

    stereo_param_path = os.path.join(stereo_depth_ros2_dir, 'cfg', 'stereo_depth_node_param.yaml')
    ov_config_path = os.path.join(stereo_depth_ros2_dir, 'config', 'openvins', 'estimator_config.yaml')
    map_param_path = os.path.join(map_manager_dir, 'cfg', 'map_param.yaml')
    dynamic_detector_param_path = os.path.join(onboard_detector_dir, 'cfg', 'dynamic_detector_param.yaml')
    yolo_detector_param_path = os.path.join(onboard_detector_dir, 'cfg', 'yolo_detector_param.yaml')
    navigation_param_path = os.path.join(navigation_runner_dir, 'cfg', 'navigation_param.yaml')
    safe_action_param_path = os.path.join(navigation_runner_dir, 'cfg', 'safe_action_param.yaml')
    rviz_config_path = os.path.join(map_manager_dir, 'rviz', 'map.rviz')

    # Optional components. YOLO requires torchvision, which is not available on
    # this Jetson torch build by default. Navigation requires torchrl, which is
    # installable, but the RL checkpoint may also be missing. Both default to
    # enabled if dependencies are present, but can be disabled via CLI.
    use_navigation_arg = DeclareLaunchArgument(
        'use_navigation', default_value='true',
        description='Launch navigation_node and safe_action_node'
    )
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo', default_value='false',
        description='Launch YOLO detector (requires torchvision)'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2'
    )

    use_navigation = LaunchConfiguration('use_navigation')
    use_yolo = LaunchConfiguration('use_yolo')
    use_rviz = LaunchConfiguration('use_rviz')

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

    # Dynamic obstacle detector
    dynamic_detector_node = Node(
        package='onboard_detector',
        executable='dynamic_detector_node',
        name='dynamic_detector_node',
        output='screen',
        parameters=[dynamic_detector_param_path]
    )

    # YOLO detector (optional, used by dynamic detector for classification)
    yolo_detector_node = Node(
        package='onboard_detector',
        executable='yolo_detector_node.py',
        name='yolo_detector_node',
        output='screen',
        parameters=[yolo_detector_param_path],
        condition=IfCondition(use_yolo)
    )

    # Safe action service (collision checking / velocity correction)
    safe_action_node = Node(
        package='navigation_runner',
        executable='safe_action_node',
        name='safe_action_node',
        output='screen',
        parameters=[safe_action_param_path],
        condition=IfCondition(use_navigation)
    )

    # Navigation node: consumes odometry, raycast, dynamic obstacles, safe action;
    # publishes cmd_vel to reach /goal_pose
    navigation_node = Node(
        package='navigation_runner',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen',
        parameters=[navigation_param_path],
        condition=IfCondition(use_navigation)
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

    # Static transform: RViz/map_manager use the 'map' frame, but OpenVINS
    # publishes its whole tree (global -> imu -> cam0/cam1) under 'global'.
    # Publish an identity map -> global so the camera/IMU frames resolve into
    # 'map' and RViz can show the TF tree, camera frusta, and point clouds.
    map_to_global_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_global_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'global'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_navigation_arg,
        use_yolo_arg,
        use_rviz_arg,
        stereo_depth_node,
        imu_node,
        openvins_node,
        occupancy_map_node,
        dynamic_detector_node,
        yolo_detector_node,
        safe_action_node,
        navigation_node,
        vio_watchdog_node,
        map_to_global_tf,
        rviz_node,
    ])
