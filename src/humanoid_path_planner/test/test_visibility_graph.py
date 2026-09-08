"""Tests for visibility edges and Euclidean A* behavior."""

from humanoid_path_planner.core.obstacle import ObstacleGeometry
from humanoid_path_planner.core.visibility_graph import shortest_path


def test_direct_path_without_obstacles():
    """An empty map must produce the direct Euclidean path."""
    result = shortest_path(
        (0.0, 0.0),
        (2.0, 0.0),
        ObstacleGeometry((), ()),
        critical_cost_multiplier=10.0,
    )

    assert result is not None
    assert result.points == ((0.0, 0.0), (2.0, 0.0))


def test_safe_detour_is_preferred_over_costly_direct_fallback():
    """A safe detour must beat the costly direct fallback edge."""
    start = (0.0, 0.0)
    goal = (2.0, 0.0)
    wall = ((0.8, -0.5), (1.2, -0.5), (1.2, 0.5), (0.8, 0.5))
    result = shortest_path(
        start,
        goal,
        ObstacleGeometry((wall,), (wall,)),
        critical_cost_multiplier=10.0,
    )

    assert result is not None
    assert (start, goal) in result.edges
    assert len(result.points) > 2
    assert any(abs(y) >= 0.5 for _, y in result.points)


def test_goal_inside_obstacle_gets_best_effort_entry():
    """A goal inside an obstacle must still receive an entry path."""
    obstacle = ((0.8, -0.5), (1.2, -0.5), (1.2, 0.5), (0.8, 0.5))
    result = shortest_path(
        (0.0, 0.0),
        (1.0, 0.0),
        ObstacleGeometry((obstacle,), (obstacle,)),
        critical_cost_multiplier=10.0,
    )

    assert result is not None
    assert result.points[0] == (0.0, 0.0)
    assert result.points[-1] == (1.0, 0.0)


def test_start_inside_obstacle_gets_best_effort_escape():
    """A localized robot inside an obstacle must still receive an exit path."""
    obstacle = ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5))
    result = shortest_path(
        (0.0, 0.0),
        (2.0, 0.0),
        ObstacleGeometry((obstacle,), (obstacle,)),
        critical_cost_multiplier=10.0,
    )

    assert result is not None
    assert result.points[0] == (0.0, 0.0)
    assert result.points[-1] == (2.0, 0.0)


def test_start_and_goal_inside_different_obstacles_get_path():
    """Best effort must escape and enter two different obstacles."""
    start_obstacle = (
        (-0.5, -0.5),
        (0.5, -0.5),
        (0.5, 0.5),
        (-0.5, 0.5),
    )
    goal_obstacle = (
        (3.5, -0.5),
        (4.5, -0.5),
        (4.5, 0.5),
        (3.5, 0.5),
    )
    result = shortest_path(
        (0.0, 0.0),
        (4.0, 0.0),
        ObstacleGeometry(
            (start_obstacle, goal_obstacle),
            (start_obstacle, goal_obstacle),
        ),
        critical_cost_multiplier=10.0,
    )

    assert result is not None
    assert result.points[0] == (0.0, 0.0)
    assert result.points[-1] == (4.0, 0.0)


def test_direct_fallback_is_returned_when_detour_is_blocked():
    """A closed obstacle cage must fall back to the direct connection."""
    start = (2.0, 0.0)
    goal = (0.0, 0.0)
    walls = (
        ((-1.1, -1.1), (-0.9, -1.1), (-0.9, 1.1), (-1.1, 1.1)),
        ((0.9, -1.1), (1.1, -1.1), (1.1, 1.1), (0.9, 1.1)),
        ((-1.1, -1.1), (1.1, -1.1), (1.1, -0.9), (-1.1, -0.9)),
        ((-1.1, 0.9), (1.1, 0.9), (1.1, 1.1), (-1.1, 1.1)),
    )
    result = shortest_path(
        start,
        goal,
        ObstacleGeometry(walls, walls),
        critical_cost_multiplier=10.0,
    )

    assert result is not None
    assert result.points == (start, goal)
