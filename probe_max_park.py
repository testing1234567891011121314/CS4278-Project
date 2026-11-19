"""At what base->object distance does the planner still find a plan?

Teleport the base in front of the object at distance d, settle arm to
travel pose, plan. Sweep d to find the reachability threshold.
"""
import sys
import time
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.grasp import GraspPlanner

ARM_TARGET = [0.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0]  # = solver.moving_pose target (head-clear)


def main():
    object_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    env = PickNavReachEnv(seed=42, object_idx=object_idx, gui=False)
    env.reset()
    for _ in range(250):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    obs = env._get_obs()
    op = np.asarray(obs["object_pos"], float)
    planner = GraspPlanner(env, robot_urdf_path="assets/fetch/fetch.urdf")

    torso = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    ds = [float(x) for x in sys.argv[3].split(",")] if len(sys.argv) > 3 else \
        [0.40, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90]
    for d in ds:
        q = np.asarray(env._get_obs()["qpos"], float).copy()
        q[0], q[1], q[2] = op[0] + d + 2.0, op[1], np.pi
        q[3] = torso
        q[6:13] = ARM_TARGET
        env._set_qpos(q)
        for _ in range(80):  # settle
            obs = env._get_obs()
            env.step(np.asarray(obs["qpos"], float))
        obs = env._get_obs()
        t0 = time.time()
        plan = planner.plan(obs)
        dt = time.time() - t0
        st = planner.last_stats
        keys = ("candidates", "tried", "opening_fail", "ik_fail", "transit_fail",
                "approach_fail", "closure_fail", "lift_fail")
        print(f"d={d:.2f} plan={'YES' if plan else 'no':3s} {dt:5.1f}s "
              + " ".join(f"{k}={st.get(k)}" for k in keys), flush=True)


if __name__ == "__main__":
    main()
