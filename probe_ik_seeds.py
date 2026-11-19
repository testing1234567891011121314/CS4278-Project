"""Does better IK seeding recover plan success at the park pose?

Settle the apple, park the base at d=0.80 facing it, take the same
candidate pregrasp targets that plan() would use, and compare the IK
success rate across seed strategies.
"""
import sys
import time
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.grasp import GraspPlanner, _ArmIK

ARM_TARGET = [0.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0]


def main():
    object_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    d = float(sys.argv[2]) if len(sys.argv) > 2 else 0.80
    env = PickNavReachEnv(seed=42, object_idx=object_idx, gui=False)
    env.reset()
    for _ in range(250):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    obs = env._get_obs()
    op = np.asarray(obs["object_pos"], float)
    q = np.asarray(env._get_obs()["qpos"], float).copy()
    q[0], q[1], q[2] = op[0] + d + 2.0, op[1], np.pi
    q[6:13] = ARM_TARGET
    env._set_qpos(q)
    for _ in range(80):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    obs = env._get_obs()

    planner = GraspPlanner(env, robot_urdf_path="assets/fetch/fetch.urdf")
    cfg = planner.config
    parked = np.asarray(obs["qpos"], float).copy()
    assert planner._configure_measured_limits(parked)
    cands = planner.sample_antipodal_grasps(points=obs["object_pc"],
                                            normals=obs["object_normals"])
    print(f"{len(cands)} candidates")
    start = parked[planner.arm_indices]
    lower, upper = planner.lower[planner.arm_indices].copy(), planner.upper[planner.arm_indices].copy()
    wrap = planner.wrap
    lower[wrap] = np.maximum(lower[wrap], start[wrap] - np.pi)
    upper[wrap] = np.minimum(upper[wrap], start[wrap] + np.pi)
    print("arm_indices:", planner.arm_indices, "wrap:", np.where(wrap)[0])
    print("start:", np.round(start, 3))
    from custom_libraries.grasp import ARM_NAMES
    names = list(ARM_NAMES)

    ik = _ArmIK(planner, parked)
    n_ok = {"parked": 0, "flip_flex": 0, "flip_roll": 0, "structured": 0, "uniform32": 0}
    n_tot = 0
    try:
        for ci, (p1, p2, centre, quat, score) in enumerate(cands[:8]):
            rot = Rotation.from_quat(quat)
            grasp_pos = centre - rot.apply(cfg.tcp_offset)
            pre_pos = grasp_pos - cfg.pregrasp_distance * rot.as_matrix()[:, 0]

            def solve(seed_list, budget):
                dl = time.monotonic() + budget
                for seed in seed_list:
                    qq = np.asarray(seed, float).copy()
                    for _ in range(4):  # same iteration budget as _ArmIK.solve
                        if time.monotonic() >= dl:
                            return None
                        for j, v in zip(ik.ids, qq):
                            p.resetJointState(ik.robot, j, float(v), physicsClientId=ik.client)
                        order = np.argsort(ik.slots)
                        r = p.calculateInverseKinematics(
                            ik.robot, ik.ee, pre_pos.tolist(), quat.tolist(),
                            lowerLimits=lower[order].tolist(), upperLimits=upper[order].tolist(),
                            jointRanges=(upper - lower)[order].tolist(),
                            restPoses=qq[order].tolist(), jointDamping=[0.03] * 7,
                            maxNumIterations=160, residualThreshold=1e-5,
                            physicsClientId=ik.client)
                        qq = np.asarray(r)[ik.slots]
                        qq[wrap] = seed[wrap] + (qq[wrap] - seed[wrap] + np.pi) % (2 * np.pi) - np.pi
                        if np.any(qq < lower) or np.any(qq > upper) or not np.isfinite(qq).all():
                            break
                        for j, v in zip(ik.ids, qq):
                            p.resetJointState(ik.robot, j, float(v), physicsClientId=ik.client)
                        st = p.getLinkState(ik.robot, ik.ee, computeForwardKinematics=True,
                                            physicsClientId=ik.client)
                        ang = (Rotation.from_quat(quat).inv() * Rotation.from_quat(st[5])).magnitude()
                        if (np.linalg.norm(np.asarray(st[4]) - pre_pos) <= cfg.position_tolerance
                                and ang <= cfg.orientation_tolerance):
                            return qq
                return None

            flex_i = names.index("wrist_flex_joint")
            roll_i = names.index("wrist_roll_joint")
            flip_flex = [start, np.clip(start - 2 * start[flex_i], lower, upper)]
            flip_roll = [start, start.copy()]
            flip_roll[1][roll_i] = start[roll_i] + np.pi
            flip_roll[1] = np.clip(flip_roll[1], lower, upper)
            structured = [start,
                          np.clip([0.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0], lower, upper),
                          np.clip([0.6, -0.5, 0.0, -1.2, 0.0, 0.5, 0.0], lower, upper),
                          np.clip([-0.3, -1.3, 0.0, -0.8, 0.0, 1.5, 0.0], lower, upper),
                          np.clip([0.3, -1.3, 0.0, -1.0, 0.0, 0.0, np.pi / 2], lower, upper),
                          np.clip([0.0, -1.0, 0.0, -1.0, 0.0, 1.0, -np.pi / 2], lower, upper)]
            uniform32 = [start] + [planner.rng.uniform(lower, upper) for _ in range(31)]

            row = []
            for label, seeds in (("parked", [start]), ("flip_flex", flip_flex),
                                 ("flip_roll", flip_roll), ("structured", structured),
                                 ("uniform32", uniform32)):
                t0 = time.monotonic()
                ok = solve(seeds, 1.0) is not None
                row.append(f"{label}={'Y' if ok else 'n'}({time.monotonic()-t0:.2f}s)")
                if ok:
                    n_ok[label] += 1
            n_tot += 1
            print(f"cand {ci} score={score:.2f} pre={np.round(pre_pos,3)} " + " ".join(row), flush=True)
    finally:
        ik.close()
    print(f"\nOUT OF {n_tot}: " + " ".join(f"{k}={v}" for k, v in n_ok.items()))


if __name__ == "__main__":
    main()
