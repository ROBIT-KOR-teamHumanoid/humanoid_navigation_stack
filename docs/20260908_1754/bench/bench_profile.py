"""물리 루프 안에서 poll() 과 나머지가 각각 몇 us 를 쓰는지 계측한다."""

import sys
import threading
import time

import rclpy

from mujoco_supervisor.supervisor_node import SupervisorNode

rclpy.init(args=sys.argv[1:])
node = SupervisorNode()

stats = {"poll_s": 0.0, "poll_n": 0, "fresh": 0}
real_poll = node.inbox.poll


def timed_poll():
    t = time.perf_counter()
    fresh = real_poll()
    stats["poll_s"] += time.perf_counter() - t
    stats["poll_n"] += 1
    stats["fresh"] += len(fresh)
    return fresh


node.inbox.poll = timed_poll

real_step = node.scene.step
stats["step_s"] = 0.0


def timed_step(steps=1):
    t = time.perf_counter()
    real_step(steps)
    stats["step_s"] += time.perf_counter() - t


node.scene.step = timed_step


def monitor():
    time.sleep(3.0)
    base = dict(stats)
    t0, s0 = time.monotonic(), node.sim_time
    time.sleep(8.0)
    elapsed = time.monotonic() - t0

    polls = stats["poll_n"] - base["poll_n"]
    poll_us = (stats["poll_s"] - base["poll_s"]) / max(polls, 1) * 1e6
    step_us = (stats["step_s"] - base["step_s"]) / max(polls, 1) * 1e6
    fresh = stats["fresh"] - base["fresh"]

    print(f"\nrtf            {(node.sim_time - s0) / elapsed:.2f}x")
    print(f"steps          {polls} in {elapsed:.1f} s "
          f"({polls / elapsed:.0f} Hz)")
    print(f"poll()         {poll_us:7.1f} us/step  "
          f"({fresh / max(polls, 1):.2f} msgs/step)")
    print(f"scene.step()   {step_us:7.1f} us/step")
    print(f"rest of loop   {1e6 / max(polls / elapsed, 1) - poll_us - step_us:7.1f} "
          f"us/step (sleep 포함)")
    node.running = False


threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
threading.Thread(target=monitor, daemon=True).start()
node.run_physics()
node.destroy_node()
rclpy.shutdown()
