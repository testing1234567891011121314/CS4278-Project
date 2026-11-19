"""Replay to lift-complete, then dump payload/arm/table geometry for carry planning."""
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver

env = PickNavReachEnv(seed=42, object_idx=7, gui=False)
env.reset()
solver = Solver(env)

steps = 0
while steps < 1500 and solver.state != 3:
    obs = env._get_obs()
    env.step(np.asarray(solver.get_action(env), dtype=float))
    steps += 1
print(f"state {solver.state} at step {steps}", flush=True)
if solver.state != 3:
    raise SystemExit

obs = env._get_obs()
q = np.asarray(obs["qpos"], float)
obj = np.asarray(obs["object_pos"], float)
np.set_printoptions(precision=3, suppress=True, linewidth=200)
print("base_xy:", q[:2] - [2.0, 0.0], "yaw:", q[2])
print("object:", obj)

robot = env.robot_id
# arm extent: AABB of all arm links + fingers + object
names = {}
for j in range(p.getNumJoints(robot, physicsClientId=env.pb_physics_client)):
    names[j] = p.getJointInfo(robot, j, physicsClientId=env.pb_physics_client)[12].decode()
arm_links = [j for j, n in names.items()
             if n in ("shoulder_link", "upper_arm_link", "elbow_flex_link",
                      "forearm_link", "wrist_flex_link", "gripper_roll_link",
                      "finger_link", "r_finger_link")
             or "finger" in n]
lows, ups = [], []
for j in arm_links:
    lo, hi = p.getAABB(robot, j, physicsClientId=env.pb_physics_client)
    lows.append(lo); ups.append(hi)
lo = np.min(lows, axis=0); hi = np.max(ups, axis=0)
olo, ohi = p.getAABB(env.object_id, -1, physicsClientId=env.pb_physics_client)
print("arm bbox lo/hi:", lo, hi)
print("obj bbox lo/hi:", np.round(olo,3), np.round(ohi,3))

# table body AABB
for i in range(p.getNumBodies(physicsClientId=env.pb_physics_client)):
    b = p.getBodyUniqueId(i, physicsClientId=env.pb_physics_client)
    nm = p.getBodyInfo(b, physicsClientId=env.pb_physics_client)[0].decode()
    if "table" in nm.lower():
        tlo = np.array([1e9]*3); thi = np.array([-1e9]*3)
        for j in range(-1, p.getNumJoints(b, physicsClientId=env.pb_physics_client)):
            try:
                lo2, hi2 = p.getAABB(b, j, physicsClientId=env.pb_physics_client)
            except Exception:
                continue
            tlo = np.minimum(tlo, lo2); thi = np.maximum(thi, hi2)
        print(f"table body {b} AABB lo/hi:", np.round(tlo,3), np.round(thi,3))

# base footprint radius
blo, bhi = p.getAABB(robot, -1, physicsClientId=env.pb_physics_client)
print("base bbox lo/hi:", np.round(blo,3), np.round(bhi,3))
print("goal:", env.goal_pos)
print("fwd from yaw:", np.round([np.cos(q[2]), np.sin(q[2])], 3))
print("base->obj vector:", np.round(obj[:2] - (q[:2] - [2.0, 0.0]), 3))
