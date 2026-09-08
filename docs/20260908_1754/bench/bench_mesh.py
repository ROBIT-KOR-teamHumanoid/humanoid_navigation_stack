"""손/머리 메시 충돌을 껐을 때 mj_step 이 얼마나 빨라지는지 잰다."""

import time

import mujoco
import numpy as np

from mujoco_supervisor.match_scene import MatchScene, RobotSpawn
from robocup_sim.apps.robot_camera import resolve_scene

ROBOT = "/home/bws/ros2_ws/src/robocup_mujoco_sim/models/ai_sapiens-k1/k1.xml"
FIELD, _ = resolve_scene("CURRENT")
SPAWNS = [
    RobotSpawn(name=f"robit_{i + 1}", x=-3.0, y=y)
    for i, y in enumerate([3.0, 1.0, -1.0, -3.0])
]


def timed(label, disable_meshes):
    scene = MatchScene(ROBOT, FIELD, SPAWNS, timestep=0.001)
    model, data = scene.model, scene.data

    disabled = 0
    if disable_meshes:
        for g in range(model.ngeom):
            if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            if not (model.geom_contype[g] or model.geom_conaffinity[g]):
                continue
            model.geom_contype[g] = 0
            model.geom_conaffinity[g] = 0
            disabled += 1

    for _ in range(300):
        mujoco.mj_step(model, data)

    steps = 4000
    t = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(model, data)
    per = (time.perf_counter() - t) / steps

    print(f"{label:<38} {per * 1e6:7.1f} us/step  "
          f"ncon={data.ncon:3d} nefc={data.nefc:4d}"
          + (f"  ({disabled} mesh geoms off)" if disable_meshes else ""))
    return per


base = timed("as-is (mesh hands + head collide)", False)
lean = timed("mesh collision disabled", True)
print(f"\nmj_step speedup: {base / lean:.2f}x  "
      f"({base * 1e6:.0f} -> {lean * 1e6:.0f} us)")

# 루프 전체 기준으로 환산: bench_loop.py 에서 commands 24 us, 관측 37 us.
other = 24.0 + 31.6 + 5.6
print(f"loop step: {base * 1e6 + other:.0f} -> {lean * 1e6 + other:.0f} us "
      f"(realtime headroom {1000 / (base * 1e6 + other):.2f}x -> "
      f"{1000 / (lean * 1e6 + other):.2f}x)")
