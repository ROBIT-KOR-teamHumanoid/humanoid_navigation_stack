"""Launch the ROBIT humanoid path follower."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return a launch description using the installed config file."""
    package_share = get_package_share_directory('humanoid_path_follower')
    config_path = os.path.join(
        package_share,
        'config',
        'path_follower.yaml',
    )
    follower = Node(
        package='humanoid_path_follower',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[config_path],
    )
    return LaunchDescription([follower])
