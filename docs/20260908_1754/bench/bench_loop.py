"""run_4match 물리 루프의 한 스텝을 구성 요소별로 잰다.

supervisor_node.run_physics 가 스텝마다 하는 일을 그대로 떼어 재현한다:
  apply_impedance_command x N  ->  mj_step  ->  publish_robot_state x N (500Hz)
                                            ->  publish_state x N (30Hz)
"""

import os
import sys
import time

import numpy as np
import rclpy

sys.setswitchinterval(0.0001)

from mujoco_supervisor.match_scene import MatchScene, RobotSpawn
from mujoco_supervisor.robot_runtime import RobotRuntime
from mujoco_supervisor.field_pixels import from_field_config
from robocup_sim.apps.robot_camera import resolve_scene

N = int(os.environ.get("N_ROBOTS", "4"))
NAMES = [f"robit_{i + 1}" for i in range(N)]
XS = [-3.0] * N
YS = [3.0, 1.0, -1.0, -3.0] * 2
TIMESTEP = 0.001

project = "/home/bws/ros2_ws/src/robocup_mujoco_sim"
robot_path = os.path.join(project, "models", "ai_sapiens-k1", "k1.xml")

field_path, field_config = resolve_scene("CURRENT")
spawns = [
    RobotSpawn(name=n, x=x, y=y, yaw_degrees=0.0)
    for n, x, y in zip(NAMES, XS, YS[:N])
]

t0 = time.perf_counter()
scene = MatchScene(robot_path, field_path, spawns, timestep=TIMESTEP)
print(f"scene build: {time.perf_counter() - t0:.2f} s, {N} robots, "
      f"nq={scene.model.nq} nv={scene.model.nv} ngeom={scene.model.ngeom}")

rclpy.init()
node = rclpy.create_node("bench")
pixels = from_field_config(field_config)

runtimes = {}
for name in NAMES:
    rt = RobotRuntime(
        node=node, name=name, scene=scene,
        imu_topic="Imu", odom_topic="ikcoordinate", vision_topic="vision",
        vision_fov_degrees=90.0, ball_enabled=True,
        localization_topic="localization", field_pixels=pixels,
    )
    rt.set_field_geometry(
        field_length=field_config.field_length,
        goal_width=field_config.goal_width,
    )
    runtimes[name] = rt

num_joints = next(iter(runtimes.values())).plant.num_joints
zeros = np.zeros(num_joints)
kp = np.full(num_joints, 50.0)
kd = np.full(num_joints, 2.0)
stamp = node.get_clock().now().to_msg()


def bench(label, fn, iterations):
    fn()  # warm up
    t = time.perf_counter()
    for _ in range(iterations):
        fn()
    per = (time.perf_counter() - t) / iterations
    print(f"{label:<34} {per * 1e6:8.1f} us")
    return per


def do_commands():
    for rt in runtimes.values():
        rt.apply_impedance_command(zeros, zeros, kp, kd)


def do_step():
    scene.step()


def do_robot_state():
    for rt in runtimes.values():
        rt.publish_robot_state(stamp)


def do_publish_state():
    for rt in runtimes.values():
        rt.publish_state()


print(f"\n--- per call, {N} robots ---")
commands = bench("apply_impedance_command x N", do_commands, 2000)
step = bench("mj_step", do_step, 2000)
robot_state = bench("publish_robot_state x N", do_robot_state, 2000)
publish_state = bench("publish_state x N", do_publish_state, 300)

# run_physics 의 실제 배분: robot_state 는 2 스텝마다(500Hz), publish_state 는
# 33 스텝마다(30Hz).
budget = TIMESTEP
per_step = commands + step + robot_state / 2 + publish_state / 33
print(f"\n--- per physics step (1 kHz budget = {budget * 1e6:.0f} us) ---")
print(f"commands            {commands * 1e6:8.1f} us  {commands / per_step:6.1%}")
print(f"mj_step             {step * 1e6:8.1f} us  {step / per_step:6.1%}")
print(f"robot_state /2      {robot_state / 2 * 1e6:8.1f} us  "
      f"{robot_state / 2 / per_step:6.1%}")
print(f"publish_state /33   {publish_state / 33 * 1e6:8.1f} us  "
      f"{publish_state / 33 / per_step:6.1%}")
print(f"{'total':<20}{per_step * 1e6:8.1f} us")
print(f"\nrealtime factor (single thread, no ROS traffic): "
      f"{budget / per_step:.2f}x")

node.destroy_node()
rclpy.shutdown()
