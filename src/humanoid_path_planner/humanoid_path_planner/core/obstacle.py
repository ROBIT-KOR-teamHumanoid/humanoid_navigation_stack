"""Obstacle models and conservative inflation for path planning."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from ..parameters import GoalObstacleParameters, ObstacleParameters

Point = tuple[float, float]
Polygon = tuple[Point, ...]


@dataclass(frozen=True)
class RoundObstacle:
    """A perceived circular obstacle before robot-footprint inflation."""

    center: Point
    radius: float

    def __post_init__(self) -> None:
        """Reject invalid obstacle radii."""
        if self.radius < 0.0:
            raise ValueError('round obstacle radius must be non-negative')


@dataclass(frozen=True)
class LayeredPolygonObstacle:
    """Critical and preferred-margin outlines of a static obstacle."""

    critical: Polygon
    margin: Polygon


@dataclass(frozen=True)
class ObstacleGeometry:
    """Polygons used for hard collision and visibility checks."""

    critical: tuple[Polygon, ...]
    margin: tuple[Polygon, ...]


class ObstacleMap:
    """Build critical and margin polygons from dynamic round obstacles."""

    def __init__(
        self,
        config: ObstacleParameters,
        static_obstacles: Sequence[LayeredPolygonObstacle] = (),
    ) -> None:
        """Store immutable obstacle configuration and static geometry."""
        self.config = config
        self.static_obstacles = tuple(static_obstacles)

    def build(
        self,
        start: Point,
        obstacles: Sequence[RoundObstacle],
    ) -> ObstacleGeometry:
        """Inflate and merge all obstacles for one planning cycle."""
        critical_circles: list[tuple[Point, float]] = []
        margin_circles: list[tuple[Point, float]] = []
        for obstacle in obstacles:
            critical_radius = obstacle.radius + self.config.robot_radius
            extra_margin = self.config.margin
            if _distance(start, obstacle.center) < self.config.near_distance:
                extra_margin += self.config.near_extra_margin
            critical_circles.append((obstacle.center, critical_radius))
            margin_circles.append(
                (obstacle.center, critical_radius + extra_margin)
            )

        critical = list(
            _merge_circles(
                critical_circles,
                self.config.merge_gap,
                self.config.polygon_vertices,
            )
        )
        margin = list(
            _merge_circles(
                margin_circles,
                self.config.merge_gap,
                self.config.polygon_vertices,
            )
        )
        critical.extend(item.critical for item in self.static_obstacles)
        margin.extend(item.margin for item in self.static_obstacles)
        return ObstacleGeometry(tuple(critical), tuple(margin))


def make_goal_obstacles(
    goal: GoalObstacleParameters,
    obstacle: ObstacleParameters,
) -> tuple[LayeredPolygonObstacle, ...]:
    """Create inflated U-shaped obstacles for both soccer goals."""
    if not goal.enabled:
        return ()

    result: list[LayeredPolygonObstacle] = []
    for side in (-1.0, 1.0):
        critical = _goal_polygon(
            side,
            goal,
            clearance=obstacle.robot_radius,
        )
        margin = _goal_polygon(
            side,
            goal,
            clearance=obstacle.robot_radius + obstacle.margin,
        )
        result.append(LayeredPolygonObstacle(critical, margin))
    return tuple(result)


def _goal_polygon(
    side: float,
    goal: GoalObstacleParameters,
    *,
    clearance: float,
) -> Polygon:
    """Return one inflated U-shaped goal polygon."""
    goal_line = goal.field_length / 2.0 + goal.goal_line_offset
    front = goal_line - clearance
    back_inner = goal_line + goal.goal_depth
    back_outer = (
        back_inner
        + goal.wall_width
        + goal.back_extension
        + clearance
    )
    inner_y = max(0.0, goal.goal_width / 2.0 - clearance)
    outer_y = goal.goal_width / 2.0 + goal.wall_width + clearance
    notch_back = max(front, back_inner - clearance)
    positive = (
        (front, -outer_y),
        (back_outer, -outer_y),
        (back_outer, outer_y),
        (front, outer_y),
        (front, inner_y),
        (notch_back, inner_y),
        (notch_back, -inner_y),
        (front, -inner_y),
    )
    return tuple((side * x, y) for x, y in positive)


def _merge_circles(
    circles: Sequence[tuple[Point, float]],
    merge_gap: float,
    vertex_count: int,
) -> tuple[Polygon, ...]:
    """Merge overlapping inflated circles using conservative convex hulls."""
    groups: list[list[int]] = []
    visited: set[int] = set()
    for first in range(len(circles)):
        if first in visited:
            continue
        visited.add(first)
        queue = [first]
        group: list[int] = []
        while queue:
            current = queue.pop()
            group.append(current)
            center, radius = circles[current]
            for candidate, (other_center, other_radius) in enumerate(circles):
                if candidate in visited:
                    continue
                if _distance(center, other_center) <= (
                    radius + other_radius + merge_gap
                ):
                    visited.add(candidate)
                    queue.append(candidate)
        groups.append(group)

    polygons: list[Polygon] = []
    for group in groups:
        points: list[Point] = []
        for index in group:
            center, radius = circles[index]
            points.extend(_circumscribed_polygon(center, radius, vertex_count))
        polygons.append(_convex_hull(points))
    return tuple(polygons)


def _circumscribed_polygon(
    center: Point,
    radius: float,
    vertex_count: int,
) -> Polygon:
    """Return a regular polygon that fully encloses the requested circle."""
    outer_radius = radius / math.cos(math.pi / vertex_count)
    angle_offset = math.pi / vertex_count
    return tuple(
        (
            center[0] + outer_radius * math.cos(
                angle_offset + math.tau * index / vertex_count
            ),
            center[1] + outer_radius * math.sin(
                angle_offset + math.tau * index / vertex_count
            ),
        )
        for index in range(vertex_count)
    )


def _convex_hull(points: Sequence[Point]) -> Polygon:
    """Return the counter-clockwise convex hull of a point collection."""
    unique = sorted(set(points))
    if len(unique) <= 1:
        return tuple(unique)

    def cross(origin: Point, first: Point, second: Point) -> float:
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower: list[Point] = []
    for point in unique:
        while (
            len(lower) >= 2
            and cross(lower[-2], lower[-1], point) <= 0.0
        ):
            lower.pop()
        lower.append(point)

    upper: list[Point] = []
    for point in reversed(unique):
        while (
            len(upper) >= 2
            and cross(upper[-2], upper[-1], point) <= 0.0
        ):
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def _distance(first: Point, second: Point) -> float:
    """Return Euclidean distance between two points."""
    return math.hypot(second[0] - first[0], second[1] - first[1])
