"""Headless probe of Astra's reference grasp.py against the real env scene.

Settle physics, park the arm in the travel pose the solver uses, then call
GraspPlanner.plan() once and report stats + plan shape.
"""
import time
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.reference.grasp import GraspPlanner, GraspFollower

TRAVEL = {6: 0.0, 7: -1.0, 8: 0.0, 9: -2.0, 10: 0.0, 11: 1.0, 12: 0.0}


def main():
    env = PickNavReachEnv(seed=42, gui=False)
    obs = env.reset()
    for _ in range(200):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))

    # Park the arm in the solver's travel pose, ramped so nothing teleports.
    obs = env._get_obs()
    qpos = np.asarray(obs["qpos"], float)
    target = qpos.copy()
    for i, v in TRAVEL.items():
        target[i] = v
    for _ in range(120):
        err = target - np.asarray(env._get_obs()["qpos"], float)
        step_cmd = np.asarray(env._get_obs()["qpos"], float) + np.clip(err, -0.05, 0.05)
        obs = env._get_obs()
        env.step(np.clip(step_cmd, env.joint_lower, env.joint_upper))
        if np.max(np.abs(err[[6, 7, 8, 9, 10, 11, 12]])) < 0.02:
            break

    planner = GraspPlanner(env, robot_urdf_path="assets/fetch/fetch.urdf")
    obs = env._get_obs()
    t0 = time.time()
    plan = planner.plan(obs)
    dt = time.time() - t0
    print(f"plan time {dt:.1f}s")
    print("stats:", planner.last_stats)
    print("sample_stats:", planner.sample_stats)
    if plan is None:
        return
    print("waypoints:", plan.waypoints.shape, "phases:", set(plan.phases))
    follower = GraspFollower(planner, plan)
    steps = 0
    while follower.status == "running" and steps < 3000:
        action = follower.step(env._get_obs())
        obs, info = env.step(action)
        steps += 1
    print(f"follower {follower.status} after {steps} steps"
          f" error={follower.error}")
    print("object z:", round(float(env._get_obs()["object_pos"][2]), 3))


if __name__ == "__main__":
    main()
