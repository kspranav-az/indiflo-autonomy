from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stereo_depth_ros2_dir = get_package_share_directory('stereo_depth_ros2')
    map_manager_dir = get_package_share_directory('map_manager')

    stereo_param_path = os.path.join(stereo_depth_ros2_dir, 'cfg', 'stereo_depth_node_param.yaml')
    odom_param_path = os.path.join(stereo_depth_ros2_dir, 'cfg', 'fake_odom_node_param.yaml')
    map_param_path = os.path.join(map_manager_dir, 'cfg', 'map_param.yaml')
    rviz_config_path = os.path.join(map_manager_dir, 'rviz', 'map.rviz')

    stereo_depth_node = Node(
        package='stereo_depth_ros2',
        executable='stereo_depth_node',
        name='stereo_depth_node',
        output='screen',
        parameters=[stereo_param_path]
    )

    fake_odom_node = Node(
        package='stereo_depth_ros2',
        executable='fake_odom_node',
        name='fake_odom_node',
        output='screen',
        parameters=[odom_param_path]
    )

    occupancy_map_node = Node(
        package='map_manager',
        executable='occupancy_map_node',
        name='map_manager_node',
        output='screen',
        parameters=[map_param_path]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        stereo_depth_node,
        fake_odom_node,
        occupancy_map_node,
        rviz_node,
    ])
