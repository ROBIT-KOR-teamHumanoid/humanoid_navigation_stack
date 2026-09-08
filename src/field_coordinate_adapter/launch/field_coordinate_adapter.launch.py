"""Launch the ROBIT field coordinate adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return a launch description using the installed config file."""
    package_share = get_package_share_directory('field_coordinate_adapter')
    config_path = os.path.join(
        package_share,
        'config',
        'field_coordinate_adapter.yaml',
    )
    adapter = Node(
        package='field_coordinate_adapter',
        executable='field_coordinate_adapter',
        name='field_coordinate_adapter',
        output='screen',
        parameters=[config_path],
    )
    return LaunchDescription([adapter])
