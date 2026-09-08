"""직선 조기 반환이 얼마나 버는지 잰다 (패키지는 건드리지 않는다).

녹화된 프레임 대부분은 로봇이 목표 10 cm 안에 있고 아무도 길을 막지 않는데,
지금은 그래도 가시성 그래프를 통째로 세운다. 시작-목표 선분이 margin 도
critical 도 건드리지 않으면 최단 경로는 그 직선이므로, 그때는 그래프를 안
세워도 같은 답이 나온다 -- 근사가 아니라 생략이다.

여기서는 그 조건을 그대로 흉내내어 (a) 판정 비용 (b) 적용률 (c) 결과가 원래
경로와 같은지를 낸다.
"""

import json
import statistics as st
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter

from humanoid_path_planner.core.obstacle import RoundObstacle
from humanoid_path_planner.core.planner import Planner, _densify
from humanoid_path_planner.core.visibility_graph import (
    segment_blocked_by_polygon,
)
from humanoid_path_planner.parameters import load_parameters as load_node_parameters

SCRATCH = Path(__file__).parent
VERTICES = int(sys.argv[2]) if len(sys.argv) > 2 else None


def parameters_for(vertices):
    config = (
        Path(get_package_share_directory("humanoid_path_planner"))
        / "config" / "path_planning.yaml"
    )
    values = yaml.safe_load(open(config))["path_planning"]["ros__parameters"]
    if vertices is not None:
        values["obstacle"]["polygon_vertices"] = vertices
    flat = {}

    def walk(node, prefix=""):
        for key, value in node.items():
            if isinstance(value, dict):
                walk(value, f"{prefix}{key}.")
            else:
                flat[f"{prefix}{key}"] = value

    walk(values)
    if not rclpy.ok():
        rclpy.init()
    node = Node(
        "bench_earlyout_params",
        parameter_overrides=[Parameter(k, value=v) for k, v in flat.items()],
    )
    try:
        return load_node_parameters(node)
    finally:
        node.destroy_node()


def straight_is_clear(start, goal, geometry) -> bool:
    """직선이 margin 과 critical 어느 쪽에도 안 걸리는가.

    critical 까지 보는 이유: 걸리기만 하고 막히지 않아도 _edge_multiplier 가
    비용에 배수를 매기므로, 그때는 플래너가 우회를 고를 수 있다. 그 경우까지
    직선으로 답하면 다른 플래너가 된다.
    """

    for polygon in geometry.margin:
        if segment_blocked_by_polygon(start, goal, polygon):
            return False
    for polygon in geometry.critical:
        if segment_blocked_by_polygon(start, goal, polygon):
            return False
    return True


frames = json.load(open(SCRATCH / sys.argv[1]))
parameters = parameters_for(VERTICES)
planner = Planner(parameters)

full_ms, early_ms = [], []
applied = 0
same = 0

for frame in frames:
    start = tuple(frame["robot"])
    goal = tuple(frame["target"])
    opponents = []
    for entry in frame["obstacles"]:
        diameter = entry["diameter"]
        opponents.append(RoundObstacle(
            tuple(entry["center"]),
            diameter / 2.0 if diameter > 0.0 else 0.075,
        ))
    ball = (
        RoundObstacle(tuple(frame["ball"]["center"]), parameters.ball.radius)
        if frame["ball"] else None
    )

    begin = time.perf_counter()
    full = planner.plan(start, goal, opponents,
                        ball=ball, avoid_ball=frame["avoid_ball"])
    full_ms.append((time.perf_counter() - begin) * 1000.0)

    # 조기 반환 경로: geometry 는 어차피 필요하다.
    begin = time.perf_counter()
    obstacles = list(opponents)
    if frame["avoid_ball"] and ball is not None:
        obstacles.append(ball)
    geometry = planner.obstacle_map.build(start, obstacles)
    if straight_is_clear(start, goal, geometry):
        path = _densify((start, goal), parameters.path_resolution)
        applied += 1
        early_ms.append((time.perf_counter() - begin) * 1000.0)
        if [[round(p[0], 4), round(p[1], 4)] for p in path] == \
           [[round(p[0], 4), round(p[1], 4)] for p in full.path]:
            same += 1
    else:
        # 막혔으면 원래대로 전체 계획을 돌린다.
        planner.plan(start, goal, opponents,
                     ball=ball, avoid_ball=frame["avoid_ball"])
        early_ms.append((time.perf_counter() - begin) * 1000.0)
        same += 1

label = VERTICES or parameters.obstacle.polygon_vertices
print(f"vertices={label}  frames={len(frames)}")
print(f"  지금        mean {st.mean(full_ms):7.1f} ms  p95 "
      f"{sorted(full_ms)[int(len(full_ms) * .95)]:7.1f} ms")
print(f"  조기반환     mean {st.mean(early_ms):7.1f} ms  p95 "
      f"{sorted(early_ms)[int(len(early_ms) * .95)]:7.1f} ms")
print(f"  적용률      {applied}/{len(frames)} ({applied / len(frames) * 100:.0f}%)")
print(f"  경로 동일   {same}/{len(frames)} ({same / len(frames) * 100:.0f}%)")
print(f"  배속        {st.mean(full_ms) / st.mean(early_ms):.1f}x")
