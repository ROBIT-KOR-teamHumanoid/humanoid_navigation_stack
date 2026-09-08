"""path_planning 이 받는 입력을 그대로 녹화한다.

3단계는 before/after 를 같은 입력으로 보여야 하므로, 합성 시나리오 대신 실제
경기에서 planner 가 본 프레임을 떠서 벤치의 입력으로 쓴다.
"""

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

NAMESPACE = os.environ.get("ROBOT", "robit_1")
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "planner_frames.json"

rclpy.init()
node = Node("record_inputs")

state = {
    "robot": None,
    "target": None,
    "obstacles": [],
    "ball": None,
    "avoid_ball": True,
}
frames = []


def point_of(marker):
    return [float(marker.pose.position.x), float(marker.pose.position.y)]


def on_robot(message: Marker) -> None:
    state["robot"] = point_of(message)


def on_target(message: Marker) -> None:
    state["target"] = point_of(message)


def on_obstacles(message: MarkerArray) -> None:
    found = []
    for marker in message.markers:
        if marker.action not in (Marker.ADD, Marker.MODIFY):
            continue
        diameter = max(float(marker.scale.x), float(marker.scale.y))
        found.append({"center": point_of(marker), "diameter": diameter})
    state["obstacles"] = found


def on_ball(message: Marker) -> None:
    state["ball"] = {
        "center": point_of(message),
        "diameter": max(float(message.scale.x), float(message.scale.y)),
    }


def on_avoid(message: Bool) -> None:
    state["avoid_ball"] = bool(message.data)


node.create_subscription(Marker, f"/{NAMESPACE}/adapter/pose_marker", on_robot, 1)
node.create_subscription(Marker, f"/{NAMESPACE}/adapter/target_marker", on_target, 1)
node.create_subscription(
    MarkerArray, f"/{NAMESPACE}/adapter/obstacle_marker", on_obstacles, 1
)
node.create_subscription(Marker, f"/{NAMESPACE}/adapter/ball_marker", on_ball, 1)
node.create_subscription(
    Bool, f"/{NAMESPACE}/ball_obstacle_active", on_avoid, 1
)

started = time.monotonic()
next_at = started
# planner 의 replan 주기(10 Hz)와 같은 간격으로 뜬다.
while time.monotonic() - started < SECONDS:
    rclpy.spin_once(node, timeout_sec=0.02)
    now = time.monotonic()
    if now < next_at:
        continue
    next_at = now + 0.1
    if state["robot"] is None or state["target"] is None:
        continue
    frames.append({
        "robot": list(state["robot"]),
        "target": list(state["target"]),
        "obstacles": [dict(o) for o in state["obstacles"]],
        "ball": dict(state["ball"]) if state["ball"] else None,
        "avoid_ball": state["avoid_ball"],
    })

with open(OUT, "w") as handle:
    json.dump(frames, handle)

counts = [len(f["obstacles"]) for f in frames]
print(f"{len(frames)} frames -> {OUT}", file=sys.stderr)
if counts:
    print(f"obstacles per frame: min {min(counts)} max {max(counts)} "
          f"mean {sum(counts) / len(counts):.1f}", file=sys.stderr)

node.destroy_node()
rclpy.shutdown()
