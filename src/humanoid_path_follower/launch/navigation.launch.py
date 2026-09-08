"""Launch the complete ROBIT humanoid navigation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Include path planning with its adapter and the path follower."""
    planner_share = get_package_share_directory('humanoid_path_planner')
    follower_share = get_package_share_directory('humanoid_path_follower')
    path_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                planner_share,
                'launch',
                'path_planning_with_adapter.launch.py',
            )
        )
    )

    path_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                follower_share,
                'launch',
                'path_follower.launch.py',
            )
        )
    )

    return LaunchDescription([path_planning, path_follower])
