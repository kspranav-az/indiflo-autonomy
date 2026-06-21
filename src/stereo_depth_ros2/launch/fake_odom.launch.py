from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('stereo_depth_ros2'),
        'cfg',
        'fake_odom_node_param.yaml'
    )

    fake_odom_node = Node(
        package='stereo_depth_ros2',
        executable='fake_odom_node',
        name='fake_odom_node',
        output='screen',
        parameters=[param_file_path]
    )

    return LaunchDescription([
        fake_odom_node,
    ])
