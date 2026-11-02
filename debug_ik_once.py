"""One-shot: build env, navigate state machine to step 59, dump candidates + IK truth."""
import numpy as np
import pybullet as p

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver
from custom_libraries.antipodal import AntipodalGraspPlanner, _quat_to_mat

env = PickNavReachEnv(seed=42)
env.reset()
solver = Solver(env)

# Fast-forward navigation exactly as main does.
for i in range(59):
    action = solver.get_action(env)
    obs, info = env.step(action)

client = env.pb_physics_client
robot_id = env.robot_id
print("qpos base xy:", obs["qpos"][:2], "yaw-ish:", obs["qpos"][2])
print("object_pos:", obs.get("object_pos"), "goal dist:", obs.get("dist_to_goal"))

planner = solver.grasp_planner
cands = planner.sample_antipodal_grasps(
    points=obs["object_pc"], normals=obs["object_normals"], num_samples=500)
print("candidates:", len(cands))
if not cands:
    raise SystemExit("sampler returned nothing")
for c in cands[:5]:
    print("  center", np.round(c[2], 3), "score", round(c[4], 3),
          "axis", np.round(c[1] - c[0], 2))

movable, arm = planner._movable_and_arm_ids(robot_id)
print("movable:", movable)
print("arm:", arm)

ee = [j for j in range(p.getNumJoints(robot_id, physicsClientId=client))
      if p.getJointInfo(robot_id, j, physicsClientId=client)[12] == b"gripper_link"]
print("ee joint->link:", ee)
ee_link = ee[0]

# Manual IK on the best candidate with sane arm rest pose.
p1, p2, center, quat, score = cands[0]
axis = (np.asarray(p2) - np.asarray(p1))
axis /= np.linalg.norm(axis)
if axis[2] < 0:
    axis = -axis
quat = planner._contact_orientations(axis[None, :])[0]
ee_target = np.asarray(center) + planner.ee_forward_offset * axis
print("ee_target:", np.round(ee_target, 3))

current = np.array([p.getJointState(robot_id, j, physicsClientId=client)[0] for j in movable])
lower, upper = current.copy(), current.copy()
arm_pos = [movable.index(j) for j in arm]
for k, j in enumerate(arm):
    info = p.getJointInfo(robot_id, j, physicsClientId=client)
    lo, hi = float(info[8]), float(info[9])
    if hi >= lo:
        lower[arm_pos[k]], upper[arm_pos[k]] = lo, hi

sol = p.calculateInverseKinematics(
    robot_id, ee_link,
    targetPosition=ee_target.tolist(), targetOrientation=quat.tolist(),
    lowerLimits=lower.tolist(), upperLimits=upper.tolist(),
    jointRanges=(upper - lower).tolist(), restPoses=current.tolist(),
    maxNumIterations=200, residualThreshold=1e-4, physicsClientId=client)
sol = np.asarray(sol)
print("sol[movable]:", np.round(sol, 3))
print("arm rest:", np.round(current[arm_pos], 3))

# FK truth
for j, v in zip(movable, current):
    p.resetJointState(robot_id, j, float(v), physicsClientId=client)
for j, v in zip(arm, sol[arm_pos]):
    p.resetJointState(robot_id, j, float(v), physicsClientId=client)
st = p.getLinkState(robot_id, ee_link, computeForwardKinematics=True, physicsClientId=client)
print("FK pos:", np.round(st[4], 3), "err:", round(float(np.linalg.norm(np.asarray(st[4]) - ee_target)), 4))
x_axis = _quat_to_mat(np.asarray(st[5]))[:, 0]
print("FK x-axis:", np.round(x_axis, 2), "vs axis:", np.round(axis, 2), "dot:", round(float(x_axis @ axis), 3))

# Is ee_target even in the arm workspace from where the base stands?
base_xy = obs["qpos"][:2]
d = np.linalg.norm(ee_target[:2] - base_xy)
print(f"horizontal distance base->ee_target: {d:.3f} m (fetch arm reach ~0.9)")
