"""Does the PRODUCTION travel pose self-collide with the head?

moving_pose target is [0,-1,0,-1,0,1,0]. Probe used elbow=-2.0.
Sweep candidate travel poses; report self-collision pairs + gripper height.
"""
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.grasp import GraspPlanner

POSES = {
    "probe(-2 elbow)": [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0],
    "production(moving_pose)": [0.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0],
    "tucked elbow=-1.5": [0.0, -1.0, 0.0, -1.5, 0.0, 1.0, 0.0],
    "elbow -1.5 lift -0.5": [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0],
    "straight forward": [0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0],
}


def main():
    env = PickNavReachEnv(seed=42, object_idx=7, gui=False)
    env.reset()
    planner = GraspPlanner(env, robot_urdf_path="assets/fetch/fetch.urdf")
    inv = {v: k for k, v in {vv[12].decode(): j for j, vv in planner.info.items()}.items()}
    inv[p.getBodyInfo(planner.robot, physicsClientId=planner.client)[0].decode()] = -1

    parked = np.asarray(env._get_obs()["qpos"], float).copy()
    for name, arm in POSES.items():
        q = parked.copy()
        q[6:13] = arm
        planner._set(q)
        bad = []
        for c in p.getClosestPoints(planner.robot, planner.robot, 0.002,
                                    physicsClientId=planner.client):
            if frozenset((c[3], c[4])) in planner.self_pairs:
                pair = tuple(sorted((inv.get(c[3], c[3]), inv.get(c[4], c[4]))))
                if pair not in [tuple(sorted(b[:2])) for b in bad]:
                    bad.append((*pair, round(c[8], 4)))
        ee = p.getLinkState(planner.robot, planner.ee, computeForwardKinematics=True,
                            physicsClientId=planner.client)
        z = ee[4][2]
        print(f"{name:26s} selfcoll={bad if bad else 'CLEAN'}  gripper_z={z:.3f}")


if __name__ == "__main__":
    main()
