from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('stereo_depth_ros2'),
        'cfg',
        'stereo_depth_node_param.yaml'
    )

    stereo_depth_node = Node(
        package='stereo_depth_ros2',
        executable='stereo_depth_node',
        name='stereo_depth_node',
        output='screen',
        parameters=[param_file_path]
    )

    return LaunchDescription([
        stereo_depth_node,
    ])
