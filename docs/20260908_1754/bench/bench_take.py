"""joint_ctrl 4 kHz 를 받는 세 가지 방법의 CPU 비용을 비교한다.

  1) rclpy.spin + 지금의 _on_command 콜백
  2) rclpy.spin + 빈 콜백        (실행기/역직렬화만)
  3) 실행기 없이 take_message 폴링 (물리 스레드가 직접 긁는 방식)

bench_pub.py 를 별도 프로세스로 같이 돌려야 한다.
"""

import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from ai_sapiens_interfaces.msg import JointImpedanceCommand

MODE = sys.argv[1]
N = int(os.environ.get("N_ROBOTS", "4"))
SECONDS = float(os.environ.get("BENCH_SECONDS", "10"))
NUM_JOINTS = 20

sys.setswitchinterval(0.0001)

rclpy.init()
node = Node("bench_take")

lock = threading.Lock()
commands = {}
received = {"n": 0}


class Command:
    def __init__(self):
        self.positions = np.zeros(NUM_JOINTS)
        self.feedforward = np.zeros(NUM_JOINTS)
        self.kp = np.zeros(NUM_JOINTS)
        self.kd = np.zeros(NUM_JOINTS)


def make_callback(name):
    # supervisor_node._on_command 과 같은 일
    def callback(message):
        received["n"] += 1
        with lock:
            command = commands[name]
            command.positions[:] = message.positions
            command.feedforward[:] = message.feedforward
            command.kp[:] = message.kp
            command.kd[:] = message.kd

    return callback


def empty_callback(message):
    received["n"] += 1


# 실행기 비용이 엔티티 수에 비례하는지 보려면 조용한 구독을 채워 넣는다
# (supervisor 는 mode_status/motion_operator, 패널 켜면 로봇당 4개가 더 붙는다).
IDLE_SUBS = int(os.environ.get("IDLE_SUBS", "0"))

subs = []
for i in range(N):
    name = f"robit_{i + 1}"
    commands[name] = Command()
    handler = empty_callback if MODE == "spin_empty" else make_callback(name)
    subs.append(
        node.create_subscription(
            JointImpedanceCommand, f"{name}/joint_ctrl", handler, 10
        )
    )

from std_msgs.msg import Float32
idle = [
    node.create_subscription(Float32, f"idle/topic_{i}", lambda m: None, 10)
    for i in range(IDLE_SUBS)
]

# 워커가 실제로 쓴 CPU 시간만 잰다 -- 벽시계는 퍼블리셔 대기를 포함한다.
def cpu_now():
    return time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)


def run_spin():
    time.sleep(1.0)
    start_msgs = received["n"]
    t0, c0 = time.monotonic(), cpu_now()
    end = t0 + SECONDS
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    return received["n"] - start_msgs, time.monotonic() - t0, cpu_now() - c0


def run_take():
    time.sleep(1.0)
    handles = [(s.handle, f"robit_{i + 1}") for i, s in enumerate(subs)]
    count = 0
    t0, c0 = time.monotonic(), cpu_now()
    end = t0 + SECONDS
    while time.monotonic() < end:
        got_any = False
        for handle, name in handles:
            with handle:
                taken = handle.take_message(JointImpedanceCommand, False)
            if taken is None:
                continue
            got_any = True
            count += 1
            message = taken[0]
            with lock:
                command = commands[name]
                command.positions[:] = message.positions
                command.feedforward[:] = message.feedforward
                command.kp[:] = message.kp
                command.kd[:] = message.kd
        if not got_any:
            # 물리 루프는 스텝 사이 sleep 이 있으니 폴링도 그만큼만 돈다
            time.sleep(0.0002)
    return count, time.monotonic() - t0, cpu_now() - c0


result = {}


def worker():
    result["value"] = run_take() if MODE == "take" else run_spin()


thread = threading.Thread(target=worker)
thread.start()
thread.join()

messages, wall, cpu = result["value"]
print(f"{MODE + '+' + str(IDLE_SUBS):<12} {messages:6d} msgs in {wall:.1f} s  "
      f"({messages / wall:6.0f}/s)  cpu={cpu:.2f} s  "
      f"=> {cpu / wall * 100:5.1f}% of a core, "
      f"{cpu / max(messages, 1) * 1e6:5.1f} us/msg")

node.destroy_node()
rclpy.shutdown()
