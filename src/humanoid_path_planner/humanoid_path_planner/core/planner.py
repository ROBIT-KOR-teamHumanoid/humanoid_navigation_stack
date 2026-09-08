"""ROS-independent orchestration of obstacle and graph planning."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .obstacle import (
    make_goal_obstacles,
    ObstacleGeometry,
    ObstacleMap,
    Point,
    RoundObstacle,
)
from .visibility_graph import shortest_path
from ..parameters import PlanningParameters


@dataclass(frozen=True)
class PlanResult:
    """Path and intermediate geometry for publication and debugging."""

    path: tuple[Point, ...]
    geometry: ObstacleGeometry
    visibility_edges: tuple[tuple[Point, Point], ...]

    @property
    def successful(self) -> bool:
        """Return whether a usable path was produced."""
        return len(self.path) >= 2


class Planner:
    """Compute shortest paths for the configured humanoid footprint."""

    def __init__(self, parameters: PlanningParameters) -> None:
        """Create static goal geometry and the reusable obstacle map."""
        self.parameters = parameters
        goals = make_goal_obstacles(
            parameters.goal_obstacle,
            parameters.obstacle,
        )
        self.obstacle_map = ObstacleMap(parameters.obstacle, goals)

    def plan(
        self,
        start: Point,
        goal: Point,
        opponents: Sequence[RoundObstacle],
        *,
        ball: RoundObstacle | None = None,
        avoid_ball: bool = False,
    ) -> PlanResult:
        """Compute a dense shortest path for the current world snapshot."""
        obstacles = list(opponents)
        if avoid_ball and ball is not None:
            obstacles.append(ball)
        geometry = self.obstacle_map.build(start, obstacles)

        if _distance(start, goal) <= 1.0e-9:
            return PlanResult((start, goal), geometry, ())

        visibility_path = shortest_path(
            start,
            goal,
            geometry,
            critical_cost_multiplier=(
                self.parameters.critical_cost_multiplier
            ),
        )
        if visibility_path is None:
            return PlanResult((), geometry, ())
        dense_path = _densify(
            visibility_path.points,
            self.parameters.path_resolution,
        )
        return PlanResult(
            dense_path,
            geometry,
            visibility_path.edges,
        )


def _densify(
    path: Sequence[Point],
    resolution: float,
) -> tuple[Point, ...]:
    """Interpolate sparse graph waypoints at a maximum spacing."""
    if len(path) < 2:
        return tuple(path)
    dense: list[Point] = [path[0]]
    for start, end in zip(path, path[1:]):
        segment_length = _distance(start, end)
        steps = max(1, math.ceil(segment_length / resolution))
        for step in range(1, steps + 1):
            fraction = step / steps
            dense.append((
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            ))
    return tuple(dense)


def _distance(first: Point, second: Point) -> float:
    """Return Euclidean distance between two points."""
    return math.hypot(second[0] - first[0], second[1] - first[1])
