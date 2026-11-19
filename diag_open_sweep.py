"""Reproduce the reference planner's pre-open collision sweep at the parked
travel pose and identify the first rejecting contact."""
import contextlib
import io
import numpy as np
import pybullet as p

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

    planner = G.GraspPlanner(env, robot_urdf_path="assets/fetch/fetch.urdf")
    cfg = planner.config
    names = {j: planner.info[j][12].decode() for j in planner.info}
    names[-1] = "base"
    body_names = {}
    for i in range(p.getNumBodies(physicsClientId=planner.client)):
        b = p.getBodyUniqueId(i, physicsClientId=planner.client)
        body_names[b] = p.getBodyInfo(b, physicsClientId=planner.client)[0].decode()

    def who_rejects(q):
        planner._set(q)
        if not np.isfinite(q).all():
            return ("limits",)
        if np.any(q < planner.lower - 1e-3) or np.any(q > planner.upper + 1e-3):
            return ("limits",)
        for body in planner.obstacles:
            for c in p.getClosestPoints(planner.robot, body, cfg.collision_margin,
                                        physicsClientId=planner.client):
                if c[3] not in planner.moving_links:
                    continue
                if body == planner.object and c[3] in planner.finger_ids \
                        and c[8] >= -cfg.contact_penetration:
                    continue
                return ("obs", body_names.get(body, body), names.get(c[3], c[3]),
                        round(float(c[8]), 4))
        for c in p.getClosestPoints(planner.robot, planner.robot, cfg.collision_margin,
                                    physicsClientId=planner.client):
            if frozenset((c[3], c[4])) in planner.self_pairs:
                return ("self", names.get(c[3], c[3]), names.get(c[4], c[4]),
                        round(float(c[8]), 4))
        return None

    planner.obstacles = [p.getBodyUniqueId(i, physicsClientId=planner.client)
                         for i in range(p.getNumBodies(physicsClientId=planner.client))]
    planner.obstacles.remove(planner.robot)

    obs = env._get_obs()
    parked = np.asarray(obs["qpos"], float)
    print("FING", parked[13:15], "lower", planner.lower[13:15],
          "upper", planner.upper[13:15])
    print("BODIES", {b: n for b, n in body_names.items()})
    print("PARKED valid?", who_rejects(parked))

    # replicate plan()'s candidate loop, stage by stage
    from scipy.spatial.transform import Rotation
    import time
    cands = planner.sample_antipodal_grasps(points=obs["object_pc"],
                                            normals=obs["object_normals"])
    print("CAND", len(cands))
    ik = G._ArmIK(planner, parked)
    start = parked[planner.arm_indices]
    lower = planner.lower[planner.arm_indices].copy()
    upper = planner.upper[planner.arm_indices].copy()
    lower[ik.owner.wrap] = np.maximum(lower[ik.owner.wrap],
                                      start[ik.owner.wrap] - np.pi)
    upper[ik.owner.wrap] = np.minimum(upper[ik.owner.wrap],
                                      start[ik.owner.wrap] + np.pi)
    for ci in range(6):
        p1, p2, centre, quat, score = cands[ci]
        width = float(np.linalg.norm(p2 - p1))
        nominal = np.full(2, (width - cfg.closed_gap) / 2)
        opened = nominal + cfg.opening_clearance / 2
        if np.any(opened > planner.upper[planner.finger_indices]):
            print(f"CAND{ci} width={width:.4f} too wide to open")
            continue
        opening = planner._full(parked, start, opened)
        w_open = None
        for s in G.edge_samples(parked, opening, 0.002):
            w_open = who_rejects(s)
            if w_open is not None:
                break
        if w_open is not None:
            print(f"CAND{ci} width={width:.4f} f_open={np.round(opened,4).tolist()} "
                  f"OPEN-SWEEP reject={w_open}")
            continue
        rot = Rotation.from_quat(quat)
        grasp_pos = centre - rot.apply(cfg.tcp_offset)
        pre_pos = grasp_pos - cfg.pregrasp_distance * rot.as_matrix()[:, 0]
        pre = ik.solve(pre_pos, quat, start, lower, upper,
                       time.monotonic() + 2.0)
        if pre is None:
            print(f"CAND{ci} width={width:.4f} PRE-IK fail")
            continue
        rrt = G.RRTConnect(lower, upper,
                           lambda arm: who_rejects(planner._full(parked, arm, opened)) is None,
                           resolution=cfg.joint_resolution, step_size=cfg.rrt_step,
                           max_iterations=cfg.rrt_iterations)
        transit = rrt.plan(start, pre, deadline=time.monotonic() + 1.0)
        if transit is None:
            print(f"CAND{ci} width={width:.4f} RRT fail")
            continue
        # closure sweep with touch=True semantics: fingers at reach config,
        # opened -> nominal; finger-object shallow contact allowed.
        reach_ok = who_rejects(planner._full(parked, pre, opened))
        close_rej = None
        for f in G.edge_samples(opened, nominal, 0.001):
            q = planner._full(parked, pre, f)
            planner._set(q)
            bad = None
            for c in p.getClosestPoints(planner.robot, planner.robot,
                                        cfg.collision_margin,
                                        physicsClientId=planner.client):
                if frozenset((c[3], c[4])) in planner.self_pairs:
                    bad = ("self", names.get(c[3], c[3]), names.get(c[4], c[4]),
                           round(float(c[8]), 4))
                    break
            if bad is None:
                for c in p.getClosestPoints(planner.robot, planner.object,
                                            cfg.collision_margin,
                                            physicsClientId=planner.client):
                    if c[3] not in planner.moving_links:
                        continue
                    if c[3] in planner.finger_ids and c[8] >= -cfg.contact_penetration:
                        continue
                    bad = ("obj", names.get(c[3], c[3]), round(float(c[8]), 4),
                           np.round(np.array(c[6]), 3).tolist())
                    break
            if bad is not None:
                close_rej = (np.round(f, 4).tolist(), bad)
                break
        print(f"CAND{ci} width={width:.4f} pre_ok={reach_ok is None} "
              f"close_rej={close_rej}")
    ik.close()


if __name__ == "__main__":
    main()
