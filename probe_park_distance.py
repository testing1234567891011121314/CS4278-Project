"""Gold test: park the base at a known distance/heading facing the object,
let physics settle, and ask the reference planner for a plan.

Isolates planner capability from parking policy.
"""
import time
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.grasp import GraspPlanner

ARM_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
             "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint",
             "wrist_roll_joint"]
ARM_TARGET = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]
ARM_LINKS = ("upperarm_roll_link", "elbow_flex_link", "forearm_roll_link",
             "wrist_flex_link", "wrist_roll_link", "gripper_link",
             "r_gripper_finger_link", "l_gripper_finger_link")


def try_park(env, planner, base_x, base_y, yaw):
    joint_ids = {p.getJointInfo(env.robot_id, j,
                                physicsClientId=env.pb_physics_client)[1].decode(): j
                 for j in range(p.getNumJoints(env.robot_id,
                                               physicsClientId=env.pb_physics_client))}
    q = np.asarray(env._get_obs()["qpos"], float).copy()
    q[0], q[1], q[2] = base_x + 2.0, base_y, yaw
    q[6:13] = ARM_TARGET
    env._set_qpos(q)
    for _ in range(160):
        obs = env._get_obs()
        q = np.asarray(obs["qpos"], float)
        t = q.copy()
        t[6:13] = ARM_TARGET
        err = t - q
        env.step(np.clip(q + np.clip(err, -0.05, 0.05),
                         env.joint_lower, env.joint_upper))
        if np.max(np.abs(err[6:13])) < 0.02:
            break
    for _ in range(40):  # settle
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    obs = env._get_obs()
    t0 = time.time()
    plan = planner.plan(obs)
    dt = time.time() - t0
    st = planner.last_stats
    print(f"park d=({base_x:.2f},{base_y:.2f}) yaw={yaw:.2f} "
          f"obj={np.round(obs['object_pos'],3)} -> "
          f"{'PLAN' if plan else 'none'} {dt:.1f}s {st}")
    return plan is not None


def main():
    env = PickNavReachEnv(seed=42, gui=False)
    env.reset()
    for _ in range(250):  # drop the object
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    op = np.asarray(env._get_obs()["object_pos"], float)
    planner = GraspPlanner(
        env, robot_urdf_path="assets/fetch/fetch.urdf",
        disabled_self_pairs=[(h, a) for h in ("head_pan_link", "head_tilt_link")
                             for a in ARM_LINKS])
    for d in (0.45, 0.55, 0.65, 0.75):
        if try_park(env, planner, op[0] + d, op[1], np.pi):
            return
    for d in (0.45, 0.55, 0.65):
        for off in (-0.15, 0.15):
            yaw = np.arctan2(op[1] - op[1], op[0] - (op[0] + d))
            if try_park(env, planner, op[0] + d, op[1] + off, yaw):
                return


if __name__ == "__main__":
    main()
