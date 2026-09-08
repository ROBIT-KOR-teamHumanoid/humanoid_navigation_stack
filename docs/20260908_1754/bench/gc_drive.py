"""GameController 창을 대신해 /gamecontroldata 를 쏜다.

GC 앱은 Qt 창이라 스크립트로 누를 수 없다. 시뮬에서 그 창이 하는 일은 상태를
골라 메시지를 내는 것뿐이므로(qnode/main_window 참고) 여기서 같은 메시지를 낸다.
gc_fanout_node 가 robotnum 만 갈아끼워 로봇별로 나눠 준다.

  INITIAL -> READY -> SET -> PLAYING 을 정해진 시각에 넘긴다.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node

from gamecontroller.msg import Gamecontroldata

STATE_NAMES = ["INITIAL", "READY", "SET", "PLAYING", "FINISHED"]

# (초, 상태) -- 시작 후 이 시각에 이 상태로 넘어간다.
SCHEDULE = [
    (0.0, 0),    # INITIAL
    (25.0, 1),   # READY   로봇이 자기 자리로 걸어간다
    (55.0, 2),   # SET     제자리에 선다
    (70.0, 3),   # PLAYING 공을 향해 움직인다
]
if os.environ.get("GC_STRAIGHT_TO_PLAYING"):
    # 프로파일링용: 부하가 걸린 상태로 바로 간다.
    SCHEDULE = [(0.0, 1), (12.0, 2), (18.0, 3)]
TOTAL_S = float(sys.argv[1]) if len(sys.argv) > 1 else 130.0

rclpy.init()
node = Node("gc_drive")
publisher = node.create_publisher(Gamecontroldata, "/gamecontroldata", 10)

message = Gamecontroldata()
message.robotnum = 1
message.position = 0
message.myteam = 21
message.myside = 0        # LEFT
message.iskickoff = 1
message.stopped = 0
message.readytime = 0
message.penalty = 0
message.secondstate = 0
message.secondinfo = []
message.message_budget = 1200

started = time.monotonic()
next_at = started
index = -1

# GC 앱과 같은 2 Hz 로 반복 발행한다 (main_window 의 500 ms 타이머).
while True:
    now = time.monotonic() - started
    if now > TOTAL_S:
        break

    wanted = 0
    for at, state in SCHEDULE:
        if now >= at:
            wanted = state
    if wanted != index:
        index = wanted
        message.state = wanted
        message.state_name = STATE_NAMES[wanted]
        print(f"[gc] {now:6.1f}s -> {STATE_NAMES[wanted]}",
              file=sys.stderr, flush=True)

    publisher.publish(message)
    # next_at 은 절대 monotonic 시각이다. started 를 다시 더하면 안 된다.
    next_at += 0.5
    sleep_for = next_at - time.monotonic()
    if sleep_for > 0:
        time.sleep(sleep_for)
    else:
        next_at = time.monotonic()

node.destroy_node()
rclpy.shutdown()
