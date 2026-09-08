"""Launch the ROBIT humanoid path planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return a launch description using the installed config file."""
    package_share = get_package_share_directory('humanoid_path_planner')
    config_path = os.path.join(
        package_share,
        'config',
        'path_planning.yaml',
    )
    planner = Node(
        package='humanoid_path_planner',
        executable='path_planning',
        name='path_planning',
        output='screen',
        parameters=[config_path],
    )
    return LaunchDescription([planner])
