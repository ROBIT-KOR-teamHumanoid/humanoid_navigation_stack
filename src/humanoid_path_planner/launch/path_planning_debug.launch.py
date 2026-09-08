"""Launch path planning with its coordinate adapter and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the existing planner stack and shut it down with RViz."""
    package_share = get_package_share_directory('humanoid_path_planner')
    default_rviz_config = os.path.join(
        package_share,
        'config',
        'rviz2.rviz',
    )
    rviz_config_argument = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file',
    )
    path_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'path_planning_with_adapter.launch.py',
            )
        )
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='RViz exited')
                )
            ],
        )
    )
    return LaunchDescription([
        rviz_config_argument,
        path_planning,
        rviz,
        shutdown_on_rviz_exit,
    ])
