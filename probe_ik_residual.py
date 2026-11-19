"""How close does DLS get to the pregrasp target? (feasibility vs convergence)"""
import sys
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.grasp import GraspPlanner, _ArmIK, ARM_NAMES

ARM_TARGET = [0.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0]


def main():
    object_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    d = float(sys.argv[2]) if len(sys.argv) > 2 else 0.80
    torso = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    env = PickNavReachEnv(seed=42, object_idx=object_idx, gui=False)
    env.reset()
    for _ in range(250):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    obs = env._get_obs()
    op = np.asarray(obs["object_pos"], float)
    q = np.asarray(env._get_obs()["qpos"], float).copy()
    q[0], q[1], q[2] = op[0] + d + 2.0, op[1], np.pi
    q[3] = torso
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
    start = parked[planner.arm_indices]
    lower, upper = planner.lower[planner.arm_indices].copy(), planner.upper[planner.arm_indices].copy()
    wrap = planner.wrap
    lower[wrap] = np.maximum(lower[wrap], start[wrap] - np.pi)
    upper[wrap] = np.minimum(upper[wrap], start[wrap] + np.pi)

    # Geometry sanity: where is the arm base relative to targets?
    ik = _ArmIK(planner, parked)
    try:
        bp, bq = p.getBasePositionAndOrientation(ik.robot, physicsClientId=ik.client)
        print("ik robot base pos:", np.round(bp, 3))
        # shoulder_pan link world pos via FK at parked
        for j, v in zip(ik.ids, start):
            p.resetJointState(ik.robot, j, float(v), physicsClientId=ik.client)
        sh = None
        for ln in range(p.getNumJoints(ik.robot, physicsClientId=ik.client)):
            st = p.getLinkState(ik.robot, ln, computeForwardKinematics=True,
                                physicsClientId=ik.client)
            nm = p.getJointInfo(ik.robot, ln, physicsClientId=ik.client)[12].decode()
            if nm == "shoulder_pan_link":
                sh = np.asarray(st[4])
        if sh is None:
            sh = np.array([-2.0, 0.0, 0.4])
        print("shoulder pos:", np.round(sh, 3))

        seen = set()
        for ci, (p1, p2, centre, quat, score) in enumerate(cands):
            rot = Rotation.from_quat(quat)
            grasp_pos = centre - rot.apply(cfg.tcp_offset)
            pre_pos = grasp_pos - cfg.pregrasp_distance * rot.as_matrix()[:, 0]
            key = tuple(np.round(pre_pos, 2))
            if key in seen:
                continue
            seen.add(key)
            if len(seen) > 6:
                break
            d_sh = np.linalg.norm(pre_pos - sh)
            app = rot.as_matrix()[:, 0]
            row = f"pre={np.round(pre_pos,3)} dist_shoulder={d_sh:.3f} app={np.round(app,2)}"
            for label, seed in (("parked", start),
                                ("random", planner.rng.uniform(lower, upper))):
                qq = np.asarray(seed, float).copy()
                order = np.argsort(ik.slots)
                err = None
                for it in range(60):
                    for j, v in zip(ik.ids, qq):
                        p.resetJointState(ik.robot, j, float(v), physicsClientId=ik.client)
                    r = p.calculateInverseKinematics(
                        ik.robot, ik.ee, pre_pos.tolist(), quat.tolist(),
                        lowerLimits=lower[order].tolist(), upperLimits=upper[order].tolist(),
                        jointRanges=(upper - lower)[order].tolist(),
                        restPoses=np.asarray(seed, float)[order].tolist(),
                        jointDamping=[0.03] * 7, maxNumIterations=160,
                        residualThreshold=1e-5, physicsClientId=ik.client)
                    qq = np.asarray(r)[ik.slots]
                    qq[wrap] = seed[wrap] + (qq[wrap] - seed[wrap] + np.pi) % (2*np.pi) - np.pi
                    if np.any(qq < lower) or np.any(qq > upper):
                        # measure error anyway by clipping for FK
                        qshow = np.clip(qq, lower, upper)
                    else:
                        qshow = qq
                    for j, v in zip(ik.ids, qshow):
                        p.resetJointState(ik.robot, j, float(v), physicsClientId=ik.client)
                    st = p.getLinkState(ik.robot, ik.ee, computeForwardKinematics=True,
                                        physicsClientId=ik.client)
                    pos_err = np.linalg.norm(np.asarray(st[4]) - pre_pos)
                    ang_err = (rot.inv() * Rotation.from_quat(st[5])).magnitude()
                    err = (pos_err, ang_err)
                    if pos_err <= cfg.position_tolerance and ang_err <= cfg.orientation_tolerance:
                        break
                row += f" | {label}: {err[0]*1000:.1f}mm {np.degrees(err[1]):.1f}deg it={it}" \
                       + ("" if np.all(qq >= lower) and np.all(qq <= upper) else " (OOR)")
            print(row, flush=True)
    finally:
        ik.close()


if __name__ == "__main__":
    main()
