"""Identify self-collision pair (6,17) at the probe travel pose.

Which links? Real penetration or a known-adjacent artifact pair?
Does the collision depend on arm pose (vary elbow_flex / shoulder_lift)?
"""
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.grasp import GraspPlanner, GraspConfig, edge_samples

ARM_TARGET = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]


def main():
    env = PickNavReachEnv(seed=42, object_idx=7, gui=False)
    env.reset()
    planner = GraspPlanner(env, robot_urdf_path="assets/fetch/fetch.urdf")

    names = {v[12].decode(): j for j, v in planner.info.items()}
    names[p.getBodyInfo(planner.robot, physicsClientId=planner.client)[0].decode()] = -1
    inv = {v: k for k, v in names.items()}
    print("link  6 =", inv.get(6), " parent:", planner.info[6][16] if 6 in planner.info else "base")
    print("link 17 =", inv.get(17), " parent:", planner.info[17][16] if 17 in planner.info else "base")
    for j in (6, 17):
        if j in planner.info:
            v = planner.info[j]
            print(f"  link {j}: type={v[2]} parent_joint_link={v[12].decode()}")

    # Is the pair even in self_pairs, and is it excluded by adjacency/clusters?
    print("(6,17) in self_pairs:", frozenset((6, 17)) in planner.self_pairs)
    print("6 in moving_links:", 6 in planner.moving_links, " 17 in moving_links:", 17 in planner.moving_links)

    parked = np.asarray(env._get_obs()["qpos"], float).copy()
    parked[6:13] = ARM_TARGET
    planner._set(parked)
    planner.obstacles = []  # self-collision only
    planner.self_pairs_backup = planner.self_pairs

    def colliding_pairs(q):
        planner._set(q)
        out = []
        for c in p.getClosestPoints(planner.robot, planner.robot, 0.002,
                                    physicsClientId=planner.client):
            if frozenset((c[3], c[4])) in planner.self_pairs_backup:
                out.append((inv.get(c[3], c[3]), inv.get(c[4], c[4]), round(c[8], 4)))
        return out

    print("\ntravel pose:", colliding_pairs(parked))
    q = parked.copy()
    q[6:13] = [0, -0.3, 0, -1.0, 0, 0.5, 0]
    print("retracted pose:", colliding_pairs(q))
    q = parked.copy()
    q[6:13] = [0, 0, 0, 0, 0, 0, 0]
    print("zero pose:", colliding_pairs(q))


if __name__ == "__main__":
    main()
