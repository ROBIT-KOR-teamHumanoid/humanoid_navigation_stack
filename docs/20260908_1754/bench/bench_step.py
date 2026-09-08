"""mj_step 안에서 시간이 어디로 가는지 MuJoCo 내장 타이머로 쪼갠다."""

import os
import sys

import mujoco
import numpy as np

from mujoco_supervisor.match_scene import MatchScene, RobotSpawn
from robocup_sim.apps.robot_camera import resolve_scene

N = int(os.environ.get("N_ROBOTS", "4"))
NAMES = [f"robit_{i + 1}" for i in range(N)]
YS = [3.0, 1.0, -1.0, -3.0, 2.0, 0.0, -2.0, -4.0]

project = "/home/bws/ros2_ws/src/robocup_mujoco_sim"
robot_path = os.path.join(project, "models", "ai_sapiens-k1", "k1.xml")
field_path, _ = resolve_scene("CURRENT")

spawns = [
    RobotSpawn(name=n, x=-3.0, y=YS[i], yaw_degrees=0.0)
    for i, n in enumerate(NAMES)
]
scene = MatchScene(robot_path, field_path, spawns, timestep=0.001)
model, data = scene.model, scene.data

print(f"robots={N} nq={model.nq} nv={model.nv} nbody={model.nbody} "
      f"ngeom={model.ngeom}")
print(f"solver={mujoco.mjtSolver(model.opt.solver).name} "
      f"iterations={model.opt.iterations} ls_iterations={model.opt.ls_iterations}")
print(f"integrator={mujoco.mjtIntegrator(model.opt.integrator).name}")
print(f"cone={mujoco.mjtCone(model.opt.cone).name} "
      f"jacobian={mujoco.mjtJacobian(model.opt.jacobian).name}")

STEPS = 3000
for _ in range(200):
    mujoco.mj_step(model, data)

TIMERS = [
    (n, getattr(mujoco.mjtTimer, n))
    for n in dir(mujoco.mjtTimer)
    if n.startswith("mjTIMER_")
]
for _name, _t in TIMERS:
    data.timer[_t].duration = 0.0
    data.timer[_t].number = 0
for _ in range(STEPS):
    mujoco.mj_step(model, data)

print(f"\n--- mj_step breakdown, us per step ({STEPS} steps) ---")
rows = []
for name, timer in TIMERS:
    entry = data.timer[timer]
    if entry.number == 0:
        continue
    per_step = entry.duration / STEPS * 1e3  # ms total -> us per step
    rows.append((per_step, name, entry.number / STEPS))

total = next((p for p, n, _ in rows if n == "mjTIMER_STEP"), 0.0)
for per_step, name, calls in sorted(rows, reverse=True):
    share = f"{per_step / total:6.1%}" if total else "     -"
    print(f"{name:<24} {per_step:8.1f} us {share}  ({calls:.1f} calls/step)")

print(f"\nncon (contacts) = {data.ncon}, nefc (constraints) = {data.nefc}")
print(f"islands = {data.nisland}")
