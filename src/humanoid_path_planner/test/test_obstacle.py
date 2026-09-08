"""Tests for circular inflation, merging, and goal geometry."""

import math

from humanoid_path_planner.core.obstacle import (
    make_goal_obstacles,
    ObstacleMap,
    RoundObstacle,
)
from humanoid_path_planner.core.visibility_graph import point_in_polygon
from humanoid_path_planner.parameters import (
    GoalObstacleParameters,
    ObstacleParameters,
)


def obstacle_parameters() -> ObstacleParameters:
    """Return compact deterministic obstacle test parameters."""
    return ObstacleParameters(
        robot_radius=0.20,
        opponent_radius_fallback=0.075,
        margin=0.10,
        near_distance=0.0,
        near_extra_margin=0.0,
        merge_gap=0.0,
        polygon_vertices=12,
    )


def goal_parameters(
    goal_line_offset: float,
) -> GoalObstacleParameters:
    """Return deterministic goal dimensions matching the package YAML."""
    return GoalObstacleParameters(
        enabled=True,
        field_length=9.0,
        goal_width=1.3,
        goal_depth=0.6,
        wall_width=0.05,
        goal_line_offset=goal_line_offset,
        back_extension=0.0,
    )


def test_round_obstacle_uses_safe_circumscribed_polygon():
    """The polygon boundary must stay outside the requested radius."""
    obstacle_map = ObstacleMap(obstacle_parameters())
    geometry = obstacle_map.build(
        (2.0, 2.0),
        [RoundObstacle((0.0, 0.0), 0.10)],
    )
    polygon = geometry.margin[0]

    assert point_in_polygon((0.399, 0.0), polygon, include_boundary=True)
    assert not point_in_polygon((0.401, 0.0), polygon, include_boundary=True)
    assert all(math.hypot(x, y) >= 0.40 for x, y in polygon)


def test_overlapping_round_obstacles_are_merged():
    """Inflated robot circles must not expose a false narrow passage."""
    obstacle_map = ObstacleMap(obstacle_parameters())
    geometry = obstacle_map.build(
        (3.0, 3.0),
        [
            RoundObstacle((0.0, 0.0), 0.10),
            RoundObstacle((0.5, 0.0), 0.10),
        ],
    )

    assert len(geometry.critical) == 1
    assert len(geometry.margin) == 1
    assert len(geometry.margin[0]) > 12


def test_goal_factory_creates_two_layered_u_shapes():
    """Both field goals must become critical and margin polygons."""
    goals = make_goal_obstacles(
        goal_parameters(0.30),
        obstacle_parameters(),
    )

    assert len(goals) == 2
    assert all(len(goal.critical) == 8 for goal in goals)
    assert max(x for x, _ in goals[1].margin) > 4.5
    assert min(x for x, _ in goals[0].margin) < -4.5


def test_zero_goal_line_offset_preserves_goal_line_geometry():
    """Zero offset must keep goals based on the original ±4.5 m lines."""
    left, right = make_goal_obstacles(
        goal_parameters(0.0),
        obstacle_parameters(),
    )

    assert math.isclose(min(x for x, _ in right.critical), 4.3)
    assert math.isclose(max(x for x, _ in left.critical), -4.3)
    assert math.isclose(min(x for x, _ in right.margin), 4.2)
    assert math.isclose(max(x for x, _ in left.margin), -4.2)


def test_goal_line_offset_translates_both_layers_outward():
    """Critical and margin polygons must translate without deformation."""
    baseline = make_goal_obstacles(
        goal_parameters(0.0),
        obstacle_parameters(),
    )
    shifted = make_goal_obstacles(
        goal_parameters(0.30),
        obstacle_parameters(),
    )

    for index, translation in ((0, -0.30), (1, 0.30)):
        for layer in ('critical', 'margin'):
            before = getattr(baseline[index], layer)
            after = getattr(shifted[index], layer)
            assert len(before) == len(after)
            for before_point, after_point in zip(before, after):
                assert math.isclose(
                    after_point[0],
                    before_point[0] + translation,
                )
                assert math.isclose(after_point[1], before_point[1])


def test_shifted_goals_are_exactly_x_symmetric():
    """The left and right shifted goal polygons must mirror each other."""
    left, right = make_goal_obstacles(
        goal_parameters(0.30),
        obstacle_parameters(),
    )

    for layer in ('critical', 'margin'):
        left_polygon = getattr(left, layer)
        right_polygon = getattr(right, layer)
        for left_point, right_point in zip(left_polygon, right_polygon):
            assert math.isclose(left_point[0], -right_point[0])
            assert math.isclose(left_point[1], right_point[1])
