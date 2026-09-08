"""녹화한 실제 입력으로 Planner.plan() 을 재생하며 시간을 잰다.

3단계의 before/after 를 같은 입력 위에서 보여주는 것이 목적이라, ROS 없이
코어만 돌린다. 속도뿐 아니라 경로가 달라졌는지도 같이 낸다 -- 빨라졌는데
다른 길로 가면 그건 개선이 아니라 다른 플래너다.

  python3 bench_planner.py frames.json [--vertices N] [--save out.json]
  python3 bench_planner.py frames.json --compare baseline.json
"""

import argparse
import json
import statistics as st
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from rclpy.node import Node
from rclpy.parameter import Parameter

from humanoid_path_planner.core.obstacle import RoundObstacle
from humanoid_path_planner.core.planner import Planner
from humanoid_path_planner.parameters import (
    load_parameters as load_node_parameters,
)


def load_parameters(vertices: int | None):
    """노드와 똑같은 경로로 파라미터를 만든다.

    parameters.load_parameters 는 ROS 노드를 받으므로, 벤치가 자기 dataclass 를
    따로 조립하면 실제와 어긋날 수 있다. 그래서 스핀하지 않는 노드를 하나 만들어
    YAML 을 override 로 밀어 넣고 같은 함수를 부른다.
    """

    config = (
        Path(get_package_share_directory("humanoid_path_planner"))
        / "config" / "path_planning.yaml"
    )
    with open(config) as handle:
        values = yaml.safe_load(handle)["path_planning"]["ros__parameters"]

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
        "bench_planner_params",
        parameter_overrides=[
            Parameter(name, value=value) for name, value in flat.items()
        ],
    )
    try:
        return load_node_parameters(node)
    finally:
        node.destroy_node()


def obstacles_of(frame) -> list:
    found = []
    for entry in frame["obstacles"]:
        diameter = entry["diameter"]
        radius = diameter / 2.0 if diameter > 0.0 else 0.075
        found.append(RoundObstacle(tuple(entry["center"]), radius))
    return found


def ball_of(frame, parameters):
    if not frame["ball"]:
        return None
    return RoundObstacle(
        tuple(frame["ball"]["center"]), parameters.ball.radius
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("frames")
    parser.add_argument("--vertices", type=int, default=None)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--save")
    parser.add_argument("--compare")
    args = parser.parse_args()

    with open(args.frames) as handle:
        frames = json.load(handle)

    parameters = load_parameters(args.vertices)
    planner = Planner(parameters)

    timings, paths, nodes = [], [], []
    for _ in range(args.repeat):
        for frame in frames:
            start = tuple(frame["robot"])
            goal = tuple(frame["target"])
            opponents = obstacles_of(frame)
            ball = ball_of(frame, parameters)

            begin = time.perf_counter()
            result = planner.plan(
                start, goal, opponents,
                ball=ball, avoid_ball=frame["avoid_ball"],
            )
            timings.append((time.perf_counter() - begin) * 1000.0)
            paths.append([[round(p[0], 4), round(p[1], 4)] for p in result.path])
            nodes.append(len(result.visibility_edges))

    label = f"vertices={args.vertices or parameters.obstacle.polygon_vertices}"
    print(f"{label}  frames={len(frames)} x{args.repeat}")
    print(f"  per plan: mean {st.mean(timings):7.1f} ms   "
          f"median {st.median(timings):7.1f} ms   "
          f"p95 {sorted(timings)[int(len(timings) * 0.95)]:7.1f} ms   "
          f"max {max(timings):7.1f} ms")
    print(f"  total {sum(timings) / 1000.0:.2f} s   "
          f"visibility edges mean {st.mean(nodes):.0f}")
    failed = sum(1 for p in paths if len(p) < 2)
    print(f"  경로 실패 {failed}/{len(paths)}")

    if args.save:
        with open(args.save, "w") as handle:
            json.dump({"paths": paths, "timings": timings}, handle)
        print(f"  -> {args.save}")

    if args.compare:
        with open(args.compare) as handle:
            other = json.load(handle)
        same = sum(1 for a, b in zip(paths, other["paths"]) if a == b)
        print(f"\n  기준 대비 경로 동일 {same}/{len(paths)} "
              f"({same / len(paths) * 100:.0f}%)")
        base = st.mean(other["timings"])
        print(f"  기준 mean {base:.1f} ms -> {st.mean(timings):.1f} ms "
              f"({base / st.mean(timings):.2f}x)")


main()
