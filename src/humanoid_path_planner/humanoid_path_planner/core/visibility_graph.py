"""ROS-independent visibility graph construction and A* search."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Sequence

from .obstacle import ObstacleGeometry, Point, Polygon

_EPSILON = 1.0e-9


@dataclass(frozen=True)
class VisibilityPath:
    """Shortest waypoint path and all accepted debug graph edges."""

    points: tuple[Point, ...]
    edges: tuple[tuple[Point, Point], ...]


@dataclass(frozen=True)
class _Node:
    """A visibility node with optional polygon ownership."""

    point: Point
    polygon_id: int = -1
    vertex_id: int = -1


def shortest_path(
    start: Point,
    goal: Point,
    geometry: ObstacleGeometry,
    *,
    critical_cost_multiplier: float,
) -> VisibilityPath | None:
    """Find a best-effort shortest path through obstacle geometry."""
    nodes = [_Node(start), _Node(goal)]
    for polygon_id, polygon in enumerate(geometry.margin):
        nodes.extend(
            _Node(point, polygon_id, vertex_id)
            for vertex_id, point in enumerate(polygon)
        )

    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    debug_edges: list[tuple[Point, Point]] = []
    for first in range(len(nodes)):
        for second in range(first + 1, len(nodes)):
            edge_multiplier = _edge_multiplier(
                first,
                second,
                nodes,
                geometry,
                critical_cost_multiplier,
            )
            if edge_multiplier is None:
                continue
            length = _distance(nodes[first].point, nodes[second].point)
            if length <= _EPSILON:
                continue
            cost = length * edge_multiplier
            adjacency[first].append((second, cost))
            adjacency[second].append((first, cost))
            debug_edges.append((nodes[first].point, nodes[second].point))

    indices = _a_star(nodes, adjacency)
    if indices is None:
        return None
    return VisibilityPath(
        points=tuple(nodes[index].point for index in indices),
        edges=tuple(debug_edges),
    )


def _edge_multiplier(
    first_index: int,
    second_index: int,
    nodes: Sequence[_Node],
    geometry: ObstacleGeometry,
    critical_cost_multiplier: float,
) -> float | None:
    """Return an edge cost multiplier, or ``None`` when blocked."""
    first = nodes[first_index]
    second = nodes[second_index]
    direct_fallback = {first_index, second_index} == {0, 1}
    adjacent = _adjacent_vertices(first, second, geometry.margin)
    if (
        first.polygon_id >= 0
        and first.polygon_id == second.polygon_id
        and not adjacent
    ):
        return None

    for polygon_id, polygon in enumerate(geometry.margin):
        allow_boundary = adjacent and first.polygon_id == polygon_id
        if not segment_blocked_by_polygon(
            first.point,
            second.point,
            polygon,
            allow_boundary_edge=allow_boundary,
        ):
            continue
        if not _special_endpoint_inside(
            first_index,
            second_index,
            nodes,
            polygon,
        ) and not direct_fallback:
            return None

    multiplier = 1.0
    for polygon in geometry.critical:
        if not segment_blocked_by_polygon(
            first.point,
            second.point,
            polygon,
            allow_boundary_edge=adjacent,
        ):
            continue
        multiplier = critical_cost_multiplier
    return multiplier


def _special_endpoint_inside(
    first_index: int,
    second_index: int,
    nodes: Sequence[_Node],
    polygon: Polygon,
) -> bool:
    """Allow start to leave or goal to enter a containing polygon."""
    return (
        first_index in (0, 1)
        and point_in_polygon(
            nodes[first_index].point,
            polygon,
            include_boundary=True,
        )
    ) or (
        second_index in (0, 1)
        and point_in_polygon(
            nodes[second_index].point,
            polygon,
            include_boundary=True,
        )
    )


def _adjacent_vertices(
    first: _Node,
    second: _Node,
    polygons: Sequence[Polygon],
) -> bool:
    """Return whether two nodes form one polygon boundary edge."""
    if first.polygon_id < 0 or first.polygon_id != second.polygon_id:
        return False
    vertex_count = len(polygons[first.polygon_id])
    difference = abs(first.vertex_id - second.vertex_id)
    return difference == 1 or difference == vertex_count - 1


def _a_star(
    nodes: Sequence[_Node],
    adjacency: Sequence[Sequence[tuple[int, float]]],
) -> list[int] | None:
    """Run Euclidean A* from node zero to node one."""
    goal_index = 1
    costs = {0: 0.0}
    parents: dict[int, int] = {}
    queue: list[tuple[float, float, int]] = [
        (_distance(nodes[0].point, nodes[goal_index].point), 0.0, 0)
    ]
    closed: set[int] = set()

    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current in closed:
            continue
        closed.add(current)
        if current == goal_index:
            path = [current]
            while path[-1] in parents:
                path.append(parents[path[-1]])
            path.reverse()
            return path

        for successor, edge_cost in adjacency[current]:
            candidate = current_cost + edge_cost
            if candidate >= costs.get(successor, math.inf):
                continue
            costs[successor] = candidate
            parents[successor] = current
            heuristic = _distance(
                nodes[successor].point,
                nodes[goal_index].point,
            )
            heapq.heappush(
                queue,
                (candidate + heuristic, candidate, successor),
            )
    return None


def point_in_polygon(
    point: Point,
    polygon: Polygon,
    *,
    include_boundary: bool,
) -> bool:
    """Return whether a point is inside a simple polygon."""
    if len(polygon) < 3:
        return False
    inside = False
    px, py = point
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(point, start, end):
            return include_boundary
        if (start[1] > py) == (end[1] > py):
            continue
        intersection_x = start[0] + (py - start[1]) * (
            end[0] - start[0]
        ) / (end[1] - start[1])
        if intersection_x > px:
            inside = not inside
    return inside


def segment_intersects_polygon(
    start: Point,
    end: Point,
    polygon: Polygon,
) -> bool:
    """Return whether a segment touches or enters a polygon."""
    if point_in_polygon(start, polygon, include_boundary=True):
        return True
    if point_in_polygon(end, polygon, include_boundary=True):
        return True
    return any(
        _segments_intersect(
            start,
            end,
            edge_start,
            polygon[(index + 1) % len(polygon)],
        )
        for index, edge_start in enumerate(polygon)
    )


def segment_blocked_by_polygon(
    start: Point,
    end: Point,
    polygon: Polygon,
    *,
    allow_boundary_edge: bool = False,
) -> bool:
    """Return whether a candidate visibility segment crosses a polygon."""
    if point_in_polygon(start, polygon, include_boundary=False):
        return True
    if point_in_polygon(end, polygon, include_boundary=False):
        return True

    for index, edge_start in enumerate(polygon):
        edge_end = polygon[(index + 1) % len(polygon)]
        if _proper_intersection(start, end, edge_start, edge_end):
            return True

        collinear = (
            abs(_cross(start, end, edge_start)) <= _EPSILON
            and abs(_cross(start, end, edge_end)) <= _EPSILON
        )
        if collinear:
            overlap = _collinear_overlap(
                start,
                end,
                edge_start,
                edge_end,
            )
            same_edge = (
                _points_close(start, edge_start)
                and _points_close(end, edge_end)
            ) or (
                _points_close(start, edge_end)
                and _points_close(end, edge_start)
            )
            if overlap > _EPSILON and not (
                allow_boundary_edge and same_edge
            ):
                return True
            continue

        touches = []
        for point in (start, end):
            if _point_on_segment(point, edge_start, edge_end):
                touches.append(point)
        for point in (edge_start, edge_end):
            if _point_on_segment(point, start, end):
                touches.append(point)
        for touch in touches:
            at_endpoint = (
                _points_close(touch, start)
                or _points_close(touch, end)
            )
            at_vertex = any(
                _points_close(touch, vertex) for vertex in polygon
            )
            if not (at_endpoint and at_vertex):
                return True

    for fraction in (1.0e-6, 0.5, 1.0 - 1.0e-6):
        sample = (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
        )
        if point_in_polygon(sample, polygon, include_boundary=False):
            return True
    return False


def _segments_intersect(
    first_start: Point,
    first_end: Point,
    second_start: Point,
    second_end: Point,
) -> bool:
    """Return whether two closed line segments intersect."""
    if _proper_intersection(
        first_start,
        first_end,
        second_start,
        second_end,
    ):
        return True
    return any((
        _point_on_segment(second_start, first_start, first_end),
        _point_on_segment(second_end, first_start, first_end),
        _point_on_segment(first_start, second_start, second_end),
        _point_on_segment(first_end, second_start, second_end),
    ))


def _proper_intersection(
    first_start: Point,
    first_end: Point,
    second_start: Point,
    second_end: Point,
) -> bool:
    """Return whether segments intersect away from their boundaries."""
    first_a = _cross(first_start, first_end, second_start)
    first_b = _cross(first_start, first_end, second_end)
    second_a = _cross(second_start, second_end, first_start)
    second_b = _cross(second_start, second_end, first_end)
    return (
        first_a * first_b < -_EPSILON
        and second_a * second_b < -_EPSILON
    )


def _point_on_segment(point: Point, start: Point, end: Point) -> bool:
    """Return whether a point lies on a closed line segment."""
    if abs(_cross(start, end, point)) > _EPSILON:
        return False
    return (
        min(start[0], end[0]) - _EPSILON
        <= point[0]
        <= max(start[0], end[0]) + _EPSILON
        and min(start[1], end[1]) - _EPSILON
        <= point[1]
        <= max(start[1], end[1]) + _EPSILON
    )


def _collinear_overlap(
    first_start: Point,
    first_end: Point,
    second_start: Point,
    second_end: Point,
) -> float:
    """Return one-dimensional overlap for collinear segments."""
    if abs(first_end[0] - first_start[0]) >= abs(
        first_end[1] - first_start[1]
    ):
        first = (first_start[0], first_end[0])
        second = (second_start[0], second_end[0])
    else:
        first = (first_start[1], first_end[1])
        second = (second_start[1], second_end[1])
    low = max(min(first), min(second))
    high = min(max(first), max(second))
    return max(0.0, high - low)


def _cross(origin: Point, first: Point, second: Point) -> float:
    """Return the signed cross product of vectors from an origin."""
    return (
        (first[0] - origin[0]) * (second[1] - origin[1])
        - (first[1] - origin[1]) * (second[0] - origin[0])
    )


def _points_close(first: Point, second: Point) -> bool:
    """Return whether points match within numeric tolerance."""
    return _distance(first, second) <= _EPSILON


def _distance(first: Point, second: Point) -> float:
    """Return Euclidean distance between two points."""
    return math.hypot(second[0] - first[0], second[1] - first[1])
