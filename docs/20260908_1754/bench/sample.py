"""path_planning CPU 와 실시간 배율을 같은 타임라인에 찍는다.

1단계의 질문은 하나다: path_planning 이 CPU 를 많이 쓰는 구간에서 시뮬이
실제로 밀리는가. 둘을 따로 재면 답이 안 나오므로 같이 샘플링한다.
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from gamecontroller.msg import Gamecontroldata

CLOCK_TICKS = os.sysconf("SC_CLK_TCK")
PERIOD_S = 2.0
TOTAL_S = float(sys.argv[1]) if len(sys.argv) > 1 else 130.0

STATE_NAMES = ["INITIAL", "READY", "SET", "PLAYING", "FINISHED"]


def pids_named(fragment: str) -> list:
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                cmdline = handle.read().decode("utf-8", "replace")
        except OSError:
            continue
        if fragment in cmdline and "sample.py" not in cmdline:
            found.append(int(entry))
    return found


def cpu_ticks(pid: int) -> float:
    try:
        with open(f"/proc/{pid}/stat") as handle:
            fields = handle.read().rsplit(") ", 1)[1].split()
    except (OSError, IndexError):
        return 0.0
    # utime, stime 은 ") " 이후 11, 12 번째 필드.
    return (float(fields[11]) + float(fields[12])) / CLOCK_TICKS


rclpy.init()
node = Node("sample")

latest = {"rtf": None, "state": None}
node.create_subscription(
    Float32, "/match/realtime_factor",
    lambda m: latest.__setitem__("rtf", float(m.data)), 1,
)
node.create_subscription(
    Gamecontroldata, "/gamecontroldata",
    lambda m: latest.__setitem__("state", int(m.state)), 1,
)

groups = {
    "planner": "humanoid_path_planner/path_planning",
    "supervisor": "mujoco_supervisor/supervisor_node",
    "taskplanner": "robocup_task_planner/task_planner_node",
    "policy": "ai_sapiens_sim2real",
}
pids = {name: pids_named(fragment) for name, fragment in groups.items()}
for name, found in pids.items():
    print(f"# {name}: {len(found)} process(es) {found}", file=sys.stderr)

previous = {
    name: sum(cpu_ticks(pid) for pid in found)
    for name, found in pids.items()
}
started = time.monotonic()
previous_at = started

print(f"{'t(s)':>6} {'state':<8} {'rtf':>6} "
      + " ".join(f"{name:>11}" for name in groups))

while time.monotonic() - started < TOTAL_S:
    deadline = time.monotonic() + PERIOD_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    now = time.monotonic()
    elapsed = now - previous_at
    previous_at = now

    columns = []
    for name, found in pids.items():
        total = sum(cpu_ticks(pid) for pid in found)
        columns.append(f"{(total - previous[name]) / elapsed * 100:10.1f}%")
        previous[name] = total

    state = latest["state"]
    label = STATE_NAMES[state] if state is not None else "-"
    rtf = latest["rtf"]
    print(f"{now - started:6.1f} {label:<8} "
          f"{rtf if rtf is not None else float('nan'):6.2f} "
          + " ".join(columns), flush=True)

node.destroy_node()
rclpy.shutdown()
