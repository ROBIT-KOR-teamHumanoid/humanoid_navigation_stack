"""제안 구현을 그대로 흉내낸다: 1 kHz 물리 루프가 스텝마다 로봇당 한 번씩
take 하고, 실행기는 joint_ctrl 을 아예 안 본다.

  spin  : 지금 방식. 스핀 스레드가 콜백으로 받고, 물리 루프는 lock 으로 읽는다.
  inbox : 제안. joint_ctrl 은 스핀 안 하는 별도 노드에 두고 물리 스레드가
          depth-1 큐에서 직접 최신 것만 긁는다.

두 경우 모두 물리 스레드가 1 kHz 로 도는 척하고, 물리 스레드와 스핀 스레드가
쓴 CPU 를 따로 잰다. bench_pub.py 를 같이 돌려야 한다.
"""

import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ai_sapiens_interfaces.msg import JointImpedanceCommand

MODE = sys.argv[1]
N = int(os.environ.get("N_ROBOTS", "4"))
SECONDS = float(os.environ.get("BENCH_SECONDS", "10"))
IDLE_SUBS = int(os.environ.get("IDLE_SUBS", "8"))  # mode_status + motion_operator
NUM_JOINTS = 20
STEP_PERIOD = 0.001

sys.setswitchinterval(0.0001)
rclpy.init()

NAMES = [f"robit_{i + 1}" for i in range(N)]


class Command:
    def __init__(self):
        self.positions = np.zeros(NUM_JOINTS)
        self.feedforward = np.zeros(NUM_JOINTS)
        self.kp = np.zeros(NUM_JOINTS)
        self.kd = np.zeros(NUM_JOINTS)


commands = {name: Command() for name in NAMES}
lock = threading.Lock()
taken = {"n": 0}

spun_node = Node("bench_inbox")

# supervisor 가 스핀 스레드에 늘 달고 있는 조용한 구독들
from std_msgs.msg import Float32
for i in range(IDLE_SUBS):
    spun_node.create_subscription(Float32, f"idle/topic_{i}", lambda m: None, 10)


def store(name, message):
    command = commands[name]
    command.positions[:] = message.positions
    command.feedforward[:] = message.feedforward
    command.kp[:] = message.kp
    command.kd[:] = message.kd


if MODE == "spin":
    def make_callback(name):
        def callback(message):
            taken["n"] += 1
            with lock:
                store(name, message)
        return callback

    for name in NAMES:
        spun_node.create_subscription(
            JointImpedanceCommand, f"{name}/joint_ctrl", make_callback(name), 10
        )
    handles = []
else:
    # 스핀되지 않는 별도 노드. 실행기가 안 보므로 대기 집합에도 안 들어간다.
    inbox_node = Node("bench_inbox_commands")
    latest_only = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1 if MODE == "inbox" else 10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    handles = []
    for name in NAMES:
        sub = inbox_node.create_subscription(
            JointImpedanceCommand,
            f"{name}/joint_ctrl",
            lambda m: None,  # 아무도 안 부른다
            latest_only,
        )
        handles.append((sub.handle, name))

cpu = {}


def cpu_now():
    return time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)


running = {"on": True}


def spin_thread():
    time.sleep(1.0)
    c0 = cpu_now()
    while running["on"]:
        rclpy.spin_once(spun_node, timeout_sec=0.01)
    cpu["spin"] = cpu_now() - c0


def physics_thread():
    time.sleep(1.0)
    c0, t0 = cpu_now(), time.monotonic()
    next_at = t0
    end = t0 + SECONDS
    steps = 0

    while time.monotonic() < end:
        if handles:
            for handle, name in handles:
                newest = None
                while True:
                    with handle:
                        got = handle.take_message(JointImpedanceCommand, False)
                    if got is None:
                        break
                    newest = got[0]
                    taken["n"] += 1
                    if MODE == "inbox":
                        break
                if newest is not None:
                    store(name, newest)
        else:
            with lock:
                for name in NAMES:
                    command = commands[name]
                    _ = command.positions[0] + command.kp[0]  # 물리 루프의 읽기

        steps += 1
        next_at += STEP_PERIOD
        sleep_for = next_at - time.monotonic()
        if sleep_for > 0.0:
            time.sleep(sleep_for)
        else:
            next_at = time.monotonic()

    wall = time.monotonic() - t0
    cpu["physics"] = cpu_now() - c0
    cpu["steps"] = steps
    cpu["wall"] = wall
    running["on"] = False


threads = [
    threading.Thread(target=spin_thread),
    threading.Thread(target=physics_thread),
]
for t in threads:
    t.start()
for t in threads:
    t.join()

wall = cpu["wall"]
rate = cpu["steps"] / wall
print(
    f"{MODE:<6} steps={cpu['steps']:6d} ({rate:6.0f} Hz of 1000)  "
    f"msgs={taken['n']:6d}  "
    f"physics={cpu['physics'] / wall * 100:5.1f}%  "
    f"spin={cpu['spin'] / wall * 100:5.1f}%  "
    f"total={(cpu['physics'] + cpu['spin']) / wall * 100:5.1f}%"
)

rclpy.shutdown()
