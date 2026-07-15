from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_viz')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'sensor_debug.rviz')

    # ------------------------------------------------------------------
    # Static TF publishers
    #   base_link -> camera / virtual_lidar / imu_link
    #   Identity for initial bring-up; refine later with real extrinsics.
    # ------------------------------------------------------------------
    tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera'],
    )

    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'virtual_lidar'],
    )

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    # ------------------------------------------------------------------
    # Sensor pipeline
    # ------------------------------------------------------------------
    midas_v4l2 = Node(
        package='midas_trt_nvargus',
        executable='midas_v4l2_node',
        name='midas_v4l2_node',
        output='screen',
    )

    depth_relay = Node(
        package='perception_viz',
        executable='depth_relay.py',
        name='depth_relay',
        output='screen',
    )

    midas_pc = Node(
        package='midas_trt_nvargus',
        executable='midas_pointcloud_node',
        name='midas_pointcloud_node',
        output='screen',
    )

    virtual_lidar = Node(
        package='midas_trt_nvargus',
        executable='virtual_lidar_from_depth_node',
        name='virtual_lidar_from_depth_node',
        output='screen',
    )

    viz_bridge = Node(
        package='midas_trt_nvargus',
        executable='viz_bridge_node',
        name='viz_bridge_node',
        output='screen',
    )

    icm20948 = Node(
        package='icm20948_ros2',
        executable='icm20948_node',
        name='icm20948_node',
        output='screen',
    )

    # ------------------------------------------------------------------
    # RViz2
    # ------------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        tf_cam,
        tf_lidar,
        tf_imu,
        midas_v4l2,
        depth_relay,
        midas_pc,
        virtual_lidar,
        viz_bridge,
        icm20948,
        rviz,
    ])
