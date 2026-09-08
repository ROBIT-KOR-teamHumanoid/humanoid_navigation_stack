"""진짜 SupervisorNode 를 띄우고 실시간 배율을 찍는다.

bench_pub.py 를 같이 돌리면 정책 1 kHz 명령이 들어올 때의 배율을, 안 돌리면
물리 루프만 있을 때의 배율을 잰다.
"""

import sys
import threading
import time

import rclpy

from mujoco_supervisor.supervisor_node import SupervisorNode


def monitor(node, seconds):
    time.sleep(2.0)  # 초기 스텝은 캐시가 차는 중이라 뺀다
    while node.running:
        t0 = time.monotonic()
        s0 = node.sim_time
        time.sleep(2.0)
        elapsed = time.monotonic() - t0
        print(f"[rtf] {(node.sim_time - s0) / elapsed:.2f}x realtime",
              file=sys.stderr, flush=True)
        seconds -= 2.0
        if seconds <= 0.0:
            node.running = False


def main():
    rclpy.init(args=sys.argv[1:])
    node = SupervisorNode()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=monitor, args=(node, 16.0), daemon=True).start()

    node.run_physics()

    node.destroy_node()
    rclpy.shutdown()


main()
