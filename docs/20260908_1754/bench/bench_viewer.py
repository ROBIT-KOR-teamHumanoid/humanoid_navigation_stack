"""viewer.sync() 한 번이 물리 스레드에서 몇 us 를 먹는지 잰다.

run_physics 는 스텝 루프 안에서 60 Hz 로 sync 를 부른다. 그 비용이 스텝
예산에 어떻게 환산되는지 보려는 것이다.
"""

import os
import time

import mujoco
import mujoco.viewer

from mujoco_supervisor.match_scene import MatchScene, RobotSpawn
from robocup_sim.apps.robot_camera import resolve_scene

ROBOT = "/home/bws/ros2_ws/src/robocup_mujoco_sim/models/ai_sapiens-k1/k1.xml"
FIELD, _ = resolve_scene("CURRENT")
SPAWNS = [
    RobotSpawn(name=f"robit_{i + 1}", x=-3.0, y=y)
    for i, y in enumerate([3.0, 1.0, -1.0, -3.0])
]

scene = MatchScene(ROBOT, FIELD, SPAWNS, timestep=0.001)
model, data = scene.model, scene.data

viewer = mujoco.viewer.launch_passive(
    model, data, show_left_ui=False, show_right_ui=False
)

# 창이 자리를 잡을 시간
for _ in range(200):
    mujoco.mj_step(model, data)
viewer.sync(state_only=True)
time.sleep(1.0)


def timed(label, fn, n):
    fn()
    t = time.perf_counter()
    for _ in range(n):
        fn()
    per = (time.perf_counter() - t) / n
    print(f"{label:<34} {per * 1e6:8.1f} us")
    return per


step = timed("mj_step", lambda: mujoco.mj_step(model, data), 2000)

# sync 만 연속으로 부르면 렌더 스레드가 아직 안 그린 상태라 값이 낮게 나온다.
# 실제처럼 스텝을 섞어 가며 잰다.
def step_and_sync():
    for _ in range(16):
        mujoco.mj_step(model, data)
    viewer.sync(state_only=True)


bundle = timed("16 steps + 1 sync(state_only)", step_and_sync, 300)
sync = bundle - 16 * step
print(f"\nsync(state_only) alone            {sync * 1e6:8.1f} us")

for hz in (60.0, 30.0, 15.0):
    per_step = sync * hz * 0.001  # 1 kHz 스텝 하나에 얹히는 몫
    print(f"  @{hz:4.0f} Hz -> {per_step * 1e6:6.1f} us/step  "
          f"(스텝 예산 1000 us 의 {per_step / 0.001:5.1%})")

viewer.close()
