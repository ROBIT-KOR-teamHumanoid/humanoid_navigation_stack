"""Launch the complete humanoid navigation stack with RViz."""

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
    """Launch navigation and shut the complete stack down with RViz."""
    package_share = get_package_share_directory('humanoid_path_follower')
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
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'navigation.launch.py',
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
        navigation,
        rviz,
        shutdown_on_rviz_exit,
    ])
