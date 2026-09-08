"""Typed ROS parameter loading for the path planning node."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass(frozen=True)
class TopicParameters:
    """Topic names used by the planner node."""

    robot: str
    target: str
    obstacles: str
    ball: str
    ball_obstacle_active: str
    path: str
    markers: str


@dataclass(frozen=True)
class FieldParameters:
    """Dimensions of the visualized soccer field."""

    length: float
    width: float
    line_width: float


@dataclass(frozen=True)
class ObstacleParameters:
    """Parameters controlling robot obstacle inflation."""

    robot_radius: float
    opponent_radius_fallback: float
    margin: float
    near_distance: float
    near_extra_margin: float
    merge_gap: float
    polygon_vertices: int


@dataclass(frozen=True)
class BallParameters:
    """Ball obstacle parameters."""

    radius: float
    avoid: bool


@dataclass(frozen=True)
class GoalObstacleParameters:
    """Static soccer goal obstacle parameters."""

    enabled: bool
    field_length: float
    goal_width: float
    goal_depth: float
    wall_width: float
    goal_line_offset: float
    back_extension: float


@dataclass(frozen=True)
class PlanningParameters:
    """Complete immutable configuration of the planner node."""

    frame_id: str
    replan_hz: float
    zero_position_is_invalid: bool
    critical_cost_multiplier: float
    path_resolution: float
    show_visibility_graph: bool
    field: FieldParameters
    topics: TopicParameters
    obstacle: ObstacleParameters
    ball: BallParameters
    goal_obstacle: GoalObstacleParameters


def load_parameters(node: 'Node') -> PlanningParameters:
    """Declare, require, validate, and return all planner parameters."""
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

    field = FieldParameters(
        length=float(value('field.length')),
        width=float(value('field.width')),
        line_width=float(value('field.line_width')),
    )
    parameters = PlanningParameters(
        frame_id=str(value('frame_id')),
        replan_hz=float(value('replan_hz')),
        zero_position_is_invalid=bool(value('zero_position_is_invalid')),
        critical_cost_multiplier=float(
            value('search.critical_cost_multiplier')
        ),
        path_resolution=float(value('search.path_resolution')),
        show_visibility_graph=bool(
            value('display.show_visibility_graph')
        ),
        field=field,
        topics=TopicParameters(
            robot=str(value('topics.robot')),
            target=str(value('topics.target')),
            obstacles=str(value('topics.obstacles')),
            ball=str(value('topics.ball')),
            ball_obstacle_active=str(
                value('topics.ball_obstacle_active')
            ),
            path=str(value('topics.path')),
            markers=str(value('topics.markers')),
        ),
        obstacle=ObstacleParameters(
            robot_radius=float(value('obstacle.robot_radius')),
            opponent_radius_fallback=float(
                value('obstacle.opponent_radius_fallback')
            ),
            margin=float(value('obstacle.margin')),
            near_distance=float(value('obstacle.near_distance')),
            near_extra_margin=float(
                value('obstacle.near_extra_margin')
            ),
            merge_gap=float(value('obstacle.merge_gap')),
            polygon_vertices=int(value('obstacle.polygon_vertices')),
        ),
        ball=BallParameters(
            radius=float(value('ball.radius')),
            avoid=bool(value('ball.avoid')),
        ),
        goal_obstacle=GoalObstacleParameters(
            enabled=bool(value('goal_obstacle.enabled')),
            field_length=field.length,
            goal_width=float(value('goal_obstacle.goal_width')),
            goal_depth=float(value('goal_obstacle.goal_depth')),
            wall_width=float(value('goal_obstacle.wall_width')),
            goal_line_offset=float(
                value('goal_obstacle.goal_line_offset')
            ),
            back_extension=float(
                value('goal_obstacle.back_extension')
            ),
        ),
    )
    _validate(parameters)
    return parameters


def _validate(parameters: PlanningParameters) -> None:
    """Raise ``ValueError`` for unsafe or nonsensical configuration."""
    if not parameters.frame_id:
        raise ValueError('frame_id must not be empty')
    if parameters.replan_hz <= 0.0:
        raise ValueError('replan_hz must be positive')
    if parameters.critical_cost_multiplier < 1.0:
        raise ValueError('critical_cost_multiplier must be at least 1')
    if parameters.path_resolution <= 0.0:
        raise ValueError('path_resolution must be positive')
    if any(
        number <= 0.0
        for number in (
            parameters.field.length,
            parameters.field.width,
            parameters.field.line_width,
        )
    ):
        raise ValueError('field dimensions and line width must be positive')
    if parameters.goal_obstacle.field_length != parameters.field.length:
        raise ValueError('goal obstacle must use the configured field length')

    obstacle_values = (
        parameters.obstacle.robot_radius,
        parameters.obstacle.opponent_radius_fallback,
        parameters.obstacle.margin,
        parameters.obstacle.near_distance,
        parameters.obstacle.near_extra_margin,
        parameters.obstacle.merge_gap,
    )
    if any(number < 0.0 for number in obstacle_values):
        raise ValueError('obstacle distances must be non-negative')
    if parameters.obstacle.polygon_vertices < 4:
        raise ValueError('obstacle.polygon_vertices must be at least 4')
    if parameters.ball.radius < 0.0:
        raise ValueError('ball.radius must be non-negative')
    if parameters.goal_obstacle.goal_line_offset < 0.0:
        raise ValueError('goal_line_offset must be non-negative')

    goal_values = (
        parameters.goal_obstacle.goal_width,
        parameters.goal_obstacle.goal_depth,
        parameters.goal_obstacle.wall_width,
        parameters.goal_obstacle.back_extension,
    )
    if any(number < 0.0 for number in goal_values):
        raise ValueError('goal obstacle dimensions must be non-negative')
    if any(not topic for topic in vars(parameters.topics).values()):
        raise ValueError('topic names must not be empty')
