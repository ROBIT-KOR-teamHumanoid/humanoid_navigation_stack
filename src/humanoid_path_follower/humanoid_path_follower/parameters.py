"""Typed ROS parameter loading for the path follower node."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass(frozen=True)
class TopicParameters:
    """Topic names used by the follower node."""

    robot_pose: str
    path: str
    cmd_vel: str
    carrot: str


@dataclass(frozen=True)
class ControllerParameters:
    """Tuning values for follow-the-carrot control."""

    carrot_index: int
    min_vel_x: float
    max_vel_x: float
    min_vel_y: float
    max_vel_y: float
    min_vel_theta: float
    max_vel_theta: float
    translation_gain: float
    rotation_kp: float
    orient_to_goal_distance: float
    position_tolerance: float
    orientation_tolerance: float
    smoothing_tau: float


@dataclass(frozen=True)
class SafetyParameters:
    """Input freshness limits for safe command publication."""

    pose_timeout: float
    path_timeout: float


@dataclass(frozen=True)
class PathFollowerParameters:
    """Complete immutable configuration of the follower node."""

    control_rate: float
    topics: TopicParameters
    controller: ControllerParameters
    safety: SafetyParameters


def load_parameters(node: 'Node') -> PathFollowerParameters:
    """Declare, require, validate, and return follower parameters."""
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.parameter import Parameter

    def value(name: str) -> Any:
        parameter = node.declare_parameter(
            name,
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        if parameter.type_ == Parameter.Type.NOT_SET:
            raise ValueError(
                f'required ROS parameter {name!r} is missing'
            )
        return parameter.value

    parameters = PathFollowerParameters(
        control_rate=float(value('control_rate')),
        topics=TopicParameters(
            robot_pose=str(value('topics.robot_pose')),
            path=str(value('topics.path')),
            cmd_vel=str(value('topics.cmd_vel')),
            carrot=str(value('topics.carrot')),
        ),
        controller=ControllerParameters(
            carrot_index=int(value('controller.carrot_index')),
            min_vel_x=float(value('controller.min_vel_x')),
            max_vel_x=float(value('controller.max_vel_x')),
            min_vel_y=float(value('controller.min_vel_y')),
            max_vel_y=float(value('controller.max_vel_y')),
            min_vel_theta=float(value('controller.min_vel_theta')),
            max_vel_theta=float(value('controller.max_vel_theta')),
            translation_gain=float(
                value('controller.translation_gain')
            ),
            rotation_kp=float(value('controller.rotation_kp')),
            orient_to_goal_distance=float(
                value('controller.orient_to_goal_distance')
            ),
            position_tolerance=float(
                value('controller.position_tolerance')
            ),
            orientation_tolerance=float(
                value('controller.orientation_tolerance')
            ),
            smoothing_tau=float(value('controller.smoothing_tau')),
        ),
        safety=SafetyParameters(
            pose_timeout=float(value('safety.pose_timeout')),
            path_timeout=float(value('safety.path_timeout')),
        ),
    )
    _validate(parameters)
    return parameters


def _validate(parameters: PathFollowerParameters) -> None:
    """Raise ``ValueError`` for unsafe or nonsensical configuration."""
    if parameters.control_rate <= 0.0:
        raise ValueError('control_rate must be positive')
    if any(not topic for topic in vars(parameters.topics).values()):
        raise ValueError('topic names must not be empty')

    controller = parameters.controller
    if controller.carrot_index < 0:
        raise ValueError('carrot_index must be non-negative')
    if controller.min_vel_x > 0.0:
        raise ValueError('min_vel_x must be non-positive')
    if controller.max_vel_x <= 0.0:
        raise ValueError('max_vel_x must be positive')
    if controller.min_vel_y > 0.0:
        raise ValueError('min_vel_y must be non-positive')
    if controller.max_vel_y <= 0.0:
        raise ValueError('max_vel_y must be positive')
    if controller.min_vel_theta > 0.0:
        raise ValueError('min_vel_theta must be non-positive')
    if controller.max_vel_theta <= 0.0:
        raise ValueError('max_vel_theta must be positive')
    if controller.translation_gain <= 0.0:
        raise ValueError('translation_gain must be positive')
    if controller.rotation_kp < 0.0:
        raise ValueError('rotation_kp must be non-negative')
    if controller.orient_to_goal_distance < 0.0:
        raise ValueError('orient_to_goal_distance must be non-negative')
    if controller.position_tolerance < 0.0:
        raise ValueError('position_tolerance must be non-negative')
    if not 0.0 <= controller.orientation_tolerance <= math.pi:
        raise ValueError('orientation_tolerance must be within [0, pi]')
    if controller.smoothing_tau < 0.0:
        raise ValueError('smoothing_tau must be non-negative')

    if parameters.safety.pose_timeout <= 0.0:
        raise ValueError('pose_timeout must be positive')
    if parameters.safety.path_timeout <= 0.0:
        raise ValueError('path_timeout must be positive')
