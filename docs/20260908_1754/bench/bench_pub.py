"""정책 N대를 흉내내 joint_ctrl 을 1 kHz 로 쏜다 (별도 프로세스)."""

import os
import sys
import time

import rclpy
from rclpy.node import Node

from ai_sapiens_interfaces.msg import JointImpedanceCommand

N = int(os.environ.get("N_ROBOTS", "4"))
RATE = float(os.environ.get("PUB_RATE", "1000"))
SECONDS = float(os.environ.get("PUB_SECONDS", "20"))
NUM_JOINTS = 23

rclpy.init()
node = Node("bench_pub")

pubs = [
    node.create_publisher(
        JointImpedanceCommand, f"robit_{i + 1}/joint_ctrl", 10
    )
    for i in range(N)
]

message = JointImpedanceCommand()
message.positions = [0.0] * NUM_JOINTS
message.feedforward = [0.0] * NUM_JOINTS
message.kp = [50.0] * NUM_JOINTS
message.kd = [2.0] * NUM_JOINTS

period = 1.0 / RATE
next_at = time.monotonic()
end_at = next_at + SECONDS
sent = 0

while time.monotonic() < end_at:
    for pub in pubs:
        pub.publish(message)
    sent += N
    next_at += period
    sleep_for = next_at - time.monotonic()
    if sleep_for > 0.0:
        time.sleep(sleep_for)
    else:
        next_at = time.monotonic()

print(f"[pub] sent {sent} messages over {SECONDS:.0f} s "
      f"({sent / SECONDS:.0f}/s across {N} topics)", file=sys.stderr)
node.destroy_node()
rclpy.shutdown()
