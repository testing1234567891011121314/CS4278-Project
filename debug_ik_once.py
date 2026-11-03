"""One-shot: navigate to step 59, then report where grasp IK targets sit in the base frame."""
import numpy as np
import pybullet as p

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver

env = PickNavReachEnv(seed=42)
env.reset()
solver = Solver(env)

for i in range(59):
    try:
        action = solver.get_action(env)
    except RuntimeError as e:
        print(f"REPLAY stopped at step {i}: {str(e)[:120]}...")
        break
    obs, info = env.step(action)

client = env.pb_physics_client
robot_id = env.robot_id

base_pos, base_orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
base_pos = np.asarray(base_pos)
R_base = np.asarray(p.getMatrixFromQuaternion(base_orn)).reshape(3, 3)
print("BASE world:", np.round(base_pos, 3))
print("BASE yaw:", round(p.getEulerFromQuaternion(base_orn)[2], 3))

obj_pos = np.asarray(obs["object_pos"])
print("OBJECT world:", np.round(obj_pos, 3))
obj_in_base = R_base.T @ (obj_pos - base_pos)
print("OBJECT in base frame (x fwd, y left):", np.round(obj_in_base, 3))

planner = solver.grasp_planner
cands = planner.sample_antipodal_grasps(points=obs["object_pc"], normals=obs["object_normals"], num_samples=500)
print("candidates:", len(cands))
for c in cands[:5]:
    ctr = np.asarray(c[2])
    axis = np.asarray(c[1]) - np.asarray(c[0])
    axis /= np.linalg.norm(axis)
    print("  center", np.round(ctr, 3), "score", round(c[4], 3),
          "axis", np.round(axis, 2),
          "center_in_base", np.round(R_base.T @ (ctr - base_pos), 3))

# FK of current gripper pose: where are the fingers actually pointing?
ee = [j for j in range(p.getNumJoints(robot_id, physicsClientId=client))
      if p.getJointInfo(robot_id, j, physicsClientId=client)[12] == b"gripper_link"][0]
st = p.getLinkState(robot_id, ee, computeForwardKinematics=True, physicsClientId=client)
ee_pos = np.asarray(st[4])
R_ee = np.asarray(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
print("EE world:", np.round(ee_pos, 3))
print("EE x-axis (fingers fwd):", np.round(R_ee[:, 0], 2),
      " y-axis (closing):", np.round(R_ee[:, 1], 2))
print("EE in base frame:", np.round(R_base.T @ (ee_pos - base_pos), 3))

# Best candidate: full IK target in base frame
p1, p2, center, quat, score = cands[0]
axis = np.asarray(p2) - np.asarray(p1)
axis /= np.linalg.norm(axis)
if axis[2] < 0:
    axis = -axis
obj_com = np.mean([np.asarray(c[2]) for c in cands[:25]], axis=0)
descent = obj_com - center
descent = descent - (descent * axis).sum() * axis
if np.linalg.norm(descent) < 1e-6:
    descent = np.array([0.0, 0.0, -1.0]) - axis[2] * axis
descent /= np.linalg.norm(descent)
ee_target = np.asarray(center) - planner.ee_forward_offset * descent
print("best candidate ee_target world:", np.round(ee_target, 3))
print("ee_target in base frame:", np.round(R_base.T @ (ee_target - base_pos), 3))
print("  (fetch arm: forward x ~0.1..0.9, |y|<0.6, z ~0-1.4 from torso)")
