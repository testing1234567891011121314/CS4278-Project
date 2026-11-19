"""Measure the torso_lift_link vs wrist_flex_link overlap at the exact
config the planner rejects, and check whether it is a real mesh collision
or a crude-STL artifact."""
import contextlib
import io
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

p.loadPlugin = lambda *a, **k: 0
_buf = io.StringIO()
with contextlib.redirect_stdout(_buf):
    from pick_nav_reach_env import PickNavReachEnv
    from custom_libraries import grasp as G

TRAVEL = {6: 0.0, 7: -1.0, 8: 0.0, 9: -2.0, 10: 0.0, 11: 1.0, 12: 0.0}


def main():
    env = PickNavReachEnv(seed=42, gui=False)
    obs = env.reset()
    for _ in range(150):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    for _ in range(120):
        obs = env._get_obs()
        q = np.asarray(obs["qpos"], float)
        t = q.copy()
        for i, v in TRAVEL.items():
            t[i] = v
        if np.max(np.abs((t - q)[6:13])) < 0.02:
            break
        env.step(np.clip(q + np.clip(t - q, -0.05, 0.05),
                         env.joint_lower, env.joint_upper))

    from solver import Solver
    solver = Solver(env)
    planner = solver.grasp_planner
    obs = env._get_obs()
    parked = np.asarray(obs["qpos"], float)
    planner.obstacles = [p.getBodyUniqueId(i, physicsClientId=planner.client)
                         for i in range(p.getNumBodies(physicsClientId=planner.client))]
    planner.obstacles.remove(planner.robot)

    names = {j: planner.info[j][12].decode() for j in planner.info}
    names[-1] = "base"
    inv = {v: k for k, v in names.items() if v != "base"}

    cands = planner.sample_antipodal_grasps(points=obs["object_pc"],
                                            normals=obs["object_normals"])
    cfg = planner.config
    ik = G._ArmIK(planner, parked)
    start = parked[planner.arm_indices]
    lower = planner.lower[planner.arm_indices].copy()
    upper = planner.upper[planner.arm_indices].copy()
    lower[ik.owner.wrap] = np.maximum(lower[ik.owner.wrap], start[ik.owner.wrap] - np.pi)
    upper[ik.owner.wrap] = np.minimum(upper[ik.owner.wrap], start[ik.owner.wrap] + np.pi)

    import time
    pre = None
    for p1, p2, centre, quat, score in cands:
        width = float(np.linalg.norm(p2 - p1))
        nominal = np.full(2, (width - cfg.closed_gap) / 2)
        opened = nominal + cfg.opening_clearance / 2
        if np.any(opened > planner.upper[planner.finger_indices]):
            continue
        rot = Rotation.from_quat(quat)
        grasp_pos = centre - rot.apply(cfg.tcp_offset)
        pre_pos = grasp_pos - cfg.pregrasp_distance * rot.as_matrix()[:, 0]
        pre = ik.solve(pre_pos, quat, start, lower, upper,
                       time.monotonic() + 2)
        if pre is not None:
            print("pre for width", round(width, 4))
            break
    print("pre", None if pre is None else np.round(pre, 3))
    if pre is None:
        return
    q = planner._full(parked, pre, opened)
    planner._set(q)
    for c in p.getClosestPoints(planner.robot, planner.robot, 0.05,
                                physicsClientId=planner.client):
        a, b = names.get(c[3], c[3]), names.get(c[4], c[4])
        if {a, b} == {"torso_lift_link", "wrist_flex_link"} or c[8] < -0.02:
            print("OVERLAP", a, b, round(float(c[8]), 4),
                  "pair_in_self_pairs", frozenset((c[3], c[4])) in planner.self_pairs)
    # world positions
    st = p.getLinkState(planner.robot, inv["wrist_flex_link"],
                        computeForwardKinematics=True, physicsClientId=planner.client)
    print("wrist world:", np.round(st[4], 3))
    lo, hi = p.getAABB(planner.robot, inv["torso_lift_link"], physicsClientId=planner.client)
    print("torso AABB lo:", np.round(lo, 3), "hi:", np.round(hi, 3))
    lo2, hi2 = p.getAABB(planner.robot, inv["forearm_roll_link"], physicsClientId=planner.client)
    print("forearm AABB lo:", np.round(lo2, 3), "hi:", np.round(hi2, 3))
    print("base x,y:", np.round(parked[:2], 3))
    ik.close()


if __name__ == "__main__":
    main()
