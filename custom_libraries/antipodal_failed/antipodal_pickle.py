"""
Antipodal Grasp Planner for Fetch robot and YCB objects in PyBullet.

  1. sample_antipodal_grasps: vectorized antipodal pair search on the caller's
     point cloud (friction cone + antiparallel normals + point-plane distance),
     geometric composite scoring, greedy center+axis diversity filter.
     Returns best-first tuples (p1, p2, center, quaternion, score) -- score is
     lower-is-better, so candidates[0] is the best geometric grasp.
  2. select_optimal_grasp: IK validation by a hand-rolled damped-least-squares
     Newton loop over the 7 arm joints only -- the stock
     calculateInverseKinematics optimizes every movable DOF and "solves"
     unreachable targets by sliding the free base joint metres sideways, so
     its numeric output cannot be trusted. Base/torso/head/fingers are
     structurally untouched; every solution is FK-verified with an axis-only
     orientation tolerance (parallel-jaw roll about the grasp axis is
     irrelevant), two strategies: "greedy_first" and "score_all".
     Returns the best reachable grasp dict, or None (never raises).
     After each call, self.ik_stats reports calls/ik_fail/fk_fail per
     strategy and per-candidate failure points in "reasons".

Grasp convention: the finger prismatic joints translate along gripper_link
y (+-0.015..0.065 measured), so the jaws CLOSE along y and the fingers
extend FORWARD along x. The quaternion maps y onto the grasp axis (p2-p1)
and x onto the approach direction; the ee sits one offset BEHIND the pads.
"""

import numpy as np
import pybullet as p

ARM_NAME_PREFIXES = (
    b"shoulder", b"upperarm", b"elbow", b"forearm", b"wrist",
)


def _mat_to_quat(mats):
    """Batched rotation matrices (N,3,3) -> quaternions (N,4) in xyzw order."""
    quats = np.empty((len(mats), 4))
    for n, m in enumerate(mats):
        tr = m.trace()
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            w, x, y, z = (0.25 * s,
                          (m[2, 1] - m[1, 2]) / s,
                          (m[0, 2] - m[2, 0]) / s,
                          (m[1, 0] - m[0, 1]) / s)
        else:
            axis = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
            i, k = (axis + 1) % 3, (axis + 2) % 3
            s = np.sqrt(1.0 + m[axis, axis] - m[i, i] - m[k, k]) * 2
            comp = [0.0, 0.0, 0.0]
            comp[axis] = 0.25 * s
            comp[i] = (m[i, axis] + m[axis, i]) / s
            comp[k] = (m[k, axis] + m[axis, k]) / s
            w = (m[k, i] - m[i, k]) / s
            x, y, z = comp
        quats[n] = (x, y, z, w)
    return quats / np.linalg.norm(quats, axis=1, keepdims=True)


def _quat_to_mat(quat):
    """xyzw quaternion -> rotation matrix (3,3)."""
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class AntipodalGraspPlanner:
    def __init__(self, gripper_max_width=0.08, physics_client_id=0,
                 mu=0.5, ee_forward_offset=0.04):
        self.rng = np.random.default_rng(0)
        self.client = physics_client_id
        self.gripper_max_width = gripper_max_width
        self.mu = mu
        self.ee_forward_offset = ee_forward_offset
        self.ik_stats = {}

    # ------------------------------------------------------------------
    # 1. Contact pair generation + ranking (no physics calls, pure point cloud)
    # ------------------------------------------------------------------

    def sample_antipodal_grasps(self, object_id=None, num_samples=500,
                                points=None, normals=None,
                                min_width=0.015, normal_tol_deg=35.0,
                                plane_tol=0.015, min_center_sep=0.03,
                                min_axis_sep_deg=30.0, outer_tol=0.03,
                                level_tol=0.1):
        """Return best-first list of (p1, p2, center, quat, score)."""
        if points is None:
            raise ValueError("Pass obs['object_pc'].")
        if normals is None:
            raise ValueError("Pass obs['object_normals'].")

        points = np.asarray(points, dtype=float)
        normals = np.asarray(normals, dtype=float)
        if len(points) < 2 or len(points) != len(normals):
            return []

        normals = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)

        com = points.mean(axis=0)
        ext = points.max(axis=0) - points.min(axis=0)

        # Surface-extremum filter: a point is on the outer shell only if no
        # sampled geometry lies beyond its tangent plane. Hollow-box models
        # (folded flaps, inner walls) put samples INSIDE the silhouette;
        # pairs built on them close the jaws on air and push the object over.
        beyond = (points[None, :, :] - points[:, None, :])
        outward = (beyond * normals[:, None, :]).sum(axis=2).max(axis=1)
        outer_shell = outward <= outer_tol

        diff = points[:, None, :] - points[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        safe_dist = np.maximum(dist, 1e-12)

        # Each filter is a named mask so the cascade count says WHICH filter
        # starved the sampler (the tilted coacd mesh sits on a knife edge).
        m_dist = (dist > min_width) & (dist <= min(
            self.gripper_max_width, np.nanmax(ext) + plane_tol))
        m_shell = m_dist & outer_shell[:, None] & outer_shell[None, :]

        axis = diff / safe_dist[:, :, None]

        # Antiparallel normals: n1 . n2 <= -cos(tol)
        n_dot = normals @ normals.T
        m_anti = m_shell & (n_dot <= -np.cos(np.radians(normal_tol_deg)))

        # Friction cone at both contacts: the contact normal must face the
        # opposing finger, i.e. -n1 within atan(mu) of the p1->p2 direction.
        cone_cos = np.cos(np.arctan(self.mu))
        n1_dir = (normals[:, None, :] * axis).sum(axis=2)    #  n1 . (p2-p1)/d
        n2_dir = -(normals[None, :, :] * axis).sum(axis=2)   # -n2 . (p2-p1)/d
        m_cone = m_anti & (n1_dir >= cone_cos) & (n2_dir >= cone_cos)

        # Point-plane contact: p2 lies on the tangent plane at p1 (and vice
        # versa) -- |n1 . (p2-p1)| must equal the full separation.
        n1_gap = np.abs((normals[:, None, :] * diff).sum(axis=2))
        n2_gap = np.abs((normals[None, :, :] * diff).sum(axis=2))
        m_plane = m_cone & (np.abs(n1_gap - dist) < plane_tol) & (np.abs(n2_gap - dist) < plane_tol)

        # Level-contact filter: a top-down pinch needs contacts at similar
        # height. A bottom-face/top-face pair is antiparallel and passes the
        # cone, but its lower finger would have to slide under the object
        # (into the table) -- ungraspable.
        dz = np.abs(diff[:, :, 2])
        valid = m_plane & (dz <= level_tol * dist)

        self.sample_stats = {
            "points": len(points),
            "outer_shell_pts": int(outer_shell.sum()),
            "m_dist": int(m_dist.sum()),
            "m_shell": int(m_shell.sum()),
            "m_anti": int(m_anti.sum()),
            "m_cone": int(m_cone.sum()),
            "m_plane": int(m_plane.sum()),
            "m_level": int(valid.sum()),
        }

        i_idx, j_idx = np.triu_indices(len(points), k=1)
        keep = valid[i_idx, j_idx]
        i_idx, j_idx = i_idx[keep], j_idx[keep]
        if len(i_idx) == 0:
            return []

        p1 = points[i_idx]
        p2 = points[j_idx]
        center = (p1 + p2) / 2.0
        sep = dist[i_idx, j_idx]
        grasp_axis = axis[i_idx, j_idx]

        # Geometric composite (lower is better):
        #  lever   - horizontal distance from COM to the grasp center line
        #  closz   - closing axis near-horizontal (|axis.z|~0): top-down pick,
        #            jaws around the sides. A vertical closing axis means
        #            pinching from above+below -- through the table.
        #  width   - prefer mid-opening: closing margin on both sides
        to_com = center - com
        horizontal = to_com - (to_com * grasp_axis).sum(axis=1, keepdims=True) * grasp_axis
        lever = np.linalg.norm(horizontal, axis=1) / max(float(np.linalg.norm(ext)), 1e-6)
        closz = np.abs(grasp_axis[:, 2])
        width = 2.0 * np.abs(sep / self.gripper_max_width - 0.5)
        score = 1.0 * lever + 0.8 * closz + 0.4 * width

        order = np.argsort(score)
        # Descent direction (ee -> object), perpendicular to the closing
        # axis: from above for center-through-COM grasps.
        descent = com - center[order]
        descent = descent - (descent * grasp_axis[order]).sum(axis=1, keepdims=True) * grasp_axis[order]
        norm = np.linalg.norm(descent, axis=1, keepdims=True)
        fallback = np.tile(np.array([0.0, 0.0, -1.0]), (len(descent), 1))
        approaches = np.where(norm > 1e-6, descent / np.maximum(norm, 1e-12), fallback)
        quats = self._contact_orientations(grasp_axis[order], approaches)

        # Greedy diversity filter on (center, axis): same-center grasps with
        # genuinely different axes (top-down vs side on a cylinder) survive.
        cos_axis_tol = np.cos(np.radians(min_axis_sep_deg))
        kept = []
        kept_c, kept_a = [], []
        for rank, o in enumerate(order):
            c, a = center[o], grasp_axis[o]
            if kept_c:
                csep = np.linalg.norm(np.array(kept_c) - c, axis=1)
                close = csep < min_center_sep
                if close.any():
                    adot = np.abs(np.array(kept_a)[close] @ a)
                    if (adot >= cos_axis_tol).any():
                        continue
            kept.append(o)
            kept_c.append(c)
            kept_a.append(a)
            if len(kept) >= num_samples:
                break

        return [
            (p1[o], p2[o], center[o], quats[k], float(score[o]))
            for k, o in enumerate(kept)
        ]

    def _contact_orientations(self, axes, approaches):
        """Quaternions whose y-axis is each grasp axis and x-axis each approach.

        The jaws close along gripper_link y, so the grasp axis (line between
        contacts) must map onto y; x is the approach direction the ee travels
        along. Roll is fully determined by both vectors.
        """
        y_axis = axes / np.linalg.norm(axes, axis=1, keepdims=True)
        x_axis = approaches - (approaches * y_axis).sum(axis=1, keepdims=True) * y_axis
        x_axis /= np.linalg.norm(x_axis, axis=1, keepdims=True)
        z_axis = np.cross(x_axis, y_axis)
        mats = np.stack([x_axis, y_axis, z_axis], axis=2)
        return _mat_to_quat(mats)

    # ------------------------------------------------------------------
    # 2. IK validation + selection
    # ------------------------------------------------------------------

    def solve_ik(self, robot_id, ee_link_index, arm_ids,
                 lower, upper, target_pos, target_rot, seed,
                 max_iterations=200, damping=0.05,
                 pos_tol=1e-4, rot_tol=1e-3):
        """Damped least squares Newton IK over the ARM JOINTS ONLY.

        The stock calculateInverseKinematics optimizes every movable DOF
        and happily "solves" an out-of-reach target by sliding the free
        mobile-base joint metres sideways, so its numeric output cannot
        be trusted. Restricting the search to the 7 arm joints makes the
        base/torso/head/fingers structurally immobile. The Jacobian is
        finite-differenced through the same FK the verifier uses.
        Backtracking guarantees monotone descent; returns the best arm
        configuration found (caller must FK-verify -- never trust these
        numbers directly).
        """
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        target_pos = np.asarray(target_pos, dtype=float)
        target_rot = np.asarray(target_rot, dtype=float)
        a = np.clip(np.asarray(seed, dtype=float), lower, upper)

        def fk_error(a_vec):
            for j, v in zip(arm_ids, a_vec):
                p.resetJointState(robot_id, j, float(v),
                                  physicsClientId=self.client)
            state = p.getLinkState(robot_id, ee_link_index,
                                   computeForwardKinematics=True,
                                   physicsClientId=self.client)
            pos = np.asarray(state[4], dtype=float)
            rot = _quat_to_mat(np.asarray(state[5], dtype=float))
            rot_err = 0.5 * (
                np.cross(rot[:, 0], target_rot[:, 0])
                + np.cross(rot[:, 1], target_rot[:, 1])
                + np.cross(rot[:, 2], target_rot[:, 2])
            )
            return np.concatenate([target_pos - pos, rot_err])

        eps = 1e-5
        err = fk_error(a)
        best_a, best_err = a.copy(), float(np.linalg.norm(err))
        for _ in range(max_iterations):
            if np.linalg.norm(err[:3]) < pos_tol and np.linalg.norm(err[3:]) < rot_tol:
                return a
            jac = np.empty((6, len(arm_ids)))
            for i in range(len(arm_ids)):
                perturbed = a.copy()
                perturbed[i] = min(perturbed[i] + eps, upper[i])
                jac[:, i] = (fk_error(perturbed) - err) / max(
                    perturbed[i] - a[i], 1e-12)
            step = -jac.T @ np.linalg.solve(
                jac @ jac.T + (damping ** 2) * np.eye(6), err)
            scale = 1.0
            for _ in range(5):
                candidate = np.clip(a + scale * step, lower, upper)
                if np.linalg.norm(fk_error(candidate)) <= np.linalg.norm(err):
                    break
                scale *= 0.5
            else:
                break
            a = np.clip(a + scale * step, lower, upper)
            err = fk_error(a)
            if np.linalg.norm(err) < best_err:
                best_a, best_err = a.copy(), float(np.linalg.norm(err))
        return best_a

    def select_optimal_grasp(self, robot_id, ee_link_index, candidates,
                             strategy="score_all", max_candidates=25,
                             pos_tol=0.01, axis_tol_deg=15.0,
                             min_center_sep=0.03, min_axis_sep_deg=30.0,
                             joint_damping=0.05, max_iterations=200,
                             support_z=None):
        """Return the best IK-verified grasp dict, or None.

        candidates: tuples (p1, p2, center, quat, score) from
        sample_antipodal_grasps. IK runs through solve_ik (arm joints
        only, hand-rolled DLS), so the base can never drift while
        "reaching"; every solution is FK-verified before acceptance.
        Rejected candidates are recorded with their failure points in
        self.ik_stats[strategy]["failures"]. joint_angles in the result
        are the 7 arm values.

        support_z: height of the surface the object rests on (e.g. the
        point cloud's minimum z). When given, the descent depth is chosen
        so the pad tips hug the surface -- pads engaging only the rim of
        a short object slip off on the lift.
        """
        if strategy not in ("greedy_first", "score_all"):
            raise ValueError("strategy must be 'greedy_first' or 'score_all'")
        if not candidates:
            self.ik_stats[strategy] = {"calls": 0, "ik_fail": 0, "fk_fail": 0,
                                       "reasons": [], "failures": []}
            return None

        movable_ids, arm_ids = self._movable_and_arm_ids(robot_id)
        if len(arm_ids) != 7:
            raise ValueError(
                f"Expected 7 arm joints, found {len(arm_ids)}: check robot URDF naming"
            )
        arm_pos = [movable_ids.index(j) for j in arm_ids]

        current = np.array([
            p.getJointState(robot_id, j, physicsClientId=self.client)[0]
            for j in movable_ids
        ])

        # This URDF claims +-314 rad for the three roll joints; a roll has
        # no identity beyond a full turn, so clamp the search to +-pi.
        arm_lower = np.empty(len(arm_ids))
        arm_upper = np.empty(len(arm_ids))
        for k, j in enumerate(arm_ids):
            info = p.getJointInfo(robot_id, j, physicsClientId=self.client)
            arm_lower[k] = np.clip(float(info[8]), -np.pi, np.pi)
            arm_upper[k] = np.clip(float(info[9]), -np.pi, np.pi)
            if arm_upper[k] <= arm_lower[k]:
                arm_lower[k] = arm_upper[k] = current[arm_pos[k]]

        # Dedup candidates spatially BEFORE spending IK calls on them.
        ranked = sorted(candidates, key=lambda c: c[4])
        picked, picked_c, picked_a = [], [], []
        cos_axis_tol = np.cos(np.radians(min_axis_sep_deg))
        for cand in ranked:
            c = np.asarray(cand[2], dtype=float)
            a = self._grasp_axis(cand)
            if picked_c:
                csep = np.linalg.norm(np.array(picked_c) - c, axis=1)
                close = csep < min_center_sep
                if close.any() and (np.abs(np.array(picked_a)[close] @ a) >= cos_axis_tol).any():
                    continue
            picked.append(cand)
            picked_c.append(c)
            picked_a.append(a)
            if len(picked) >= max_candidates:
                break

        stats = {"calls": 0, "ik_fail": 0, "fk_fail": 0, "reasons": [],
                 "failures": []}
        self.ik_stats[strategy] = stats
        passing = []

        # Multi-start Newton needs diverse basins. wrist_flex POSITIVE points
        # the fingers down (FK: -1.57 leaves gripper x pointing UP, and
        # negative-wrist starts only reach "x down" by wrapping through +-pi);
        # shoulder_lift runs both signs -- the verified solution lives at
        # negative lift, which none of the original seeds sampled.
        seed_arm_poses = [
            np.array([0.0, 1.0, 0.0, -1.6, 0.0, 1.0, 0.0]),
            np.array([0.0, 1.3, 0.0, -1.5, 0.0, 1.5, 0.0]),
            np.array([0.0, -0.6, 1.1, 0.9, -0.7, 1.9, -1.7]),
            np.array([-0.4, -0.6, -1.1, 1.6, -2.8, -1.0, -1.8]),
            current[arm_pos],
        ]

        for p1, p2, center, quat, score in picked:
            center = np.asarray(center, dtype=float)
            axis = self._grasp_axis((p1, p2, center, quat, score))
            # Parallel jaws are symmetric: fix the sign for determinism.
            if axis[2] < 0:
                axis = -axis
            # Descent direction: the direction the ee travels toward the
            # object, perpendicular to the closing axis. Always downward:
            # com-center points UP for contacts below the object's middle,
            # which parks the hand body under the table (motor stall one
            # finger-length short of target). A vertical closing axis has
            # no downward approach (jaws above and below the object -- one
            # finger through the table); reject it.
            descent = np.array([0.0, 0.0, -1.0])
            descent = descent - (descent * axis).sum() * axis
            if np.linalg.norm(descent) < 1e-3:
                stats["failures"].append({
                    "center": np.round(center, 3).tolist(),
                    "reason": "vertical closing axis: no downward approach",
                })
                if len(stats["reasons"]) < 5:
                    stats["reasons"].append(
                        f"vertical closing axis at center {np.round(center, 3)}")
                stats["ik_fail"] += 1
                continue
            descent /= np.linalg.norm(descent)
            quat = self._contact_orientations(axis[None, :], descent[None, :])[0]
            # The gripper's +x (pad tips, ee_forward_offset ahead of the
            # wrist) points along descent, so the wrist sits depth ABOVE
            # center and the tips land at center_z + depth - offset. The
            # nominal depth (offset) puts tips at the contact centre; for a
            # short object on a table that engages only the rim and slips
            # off on the lift, so when support_z is known drop the tips to
            # ~5 mm above the surface (never below it).
            if support_z is None:
                depth = self.ee_forward_offset
            else:
                depth = max(0.0, self.ee_forward_offset + support_z + 0.005 - center[2])
            ee_target = center - depth * descent

            target_rot = _quat_to_mat(np.asarray(quat, dtype=float))

            best = None  # (pos_err, axis_err_deg, arm_sol)
            try:
                for seed_arm in seed_arm_poses:
                    sol = self.solve_ik(
                        robot_id, ee_link_index, arm_ids,
                        arm_lower, arm_upper, ee_target, target_rot, seed_arm,
                        max_iterations=max_iterations, damping=joint_damping,
                    )
                    stats["calls"] += 1
                    pos_err, axis_err, approach_err = self._fk_verify(
                        robot_id, ee_link_index, movable_ids,
                        arm_ids, arm_pos, current, sol,
                        ee_target, quat, axis, descent)
                    if best is None or (pos_err, axis_err) < (best[0], best[1]):
                        best = (pos_err, axis_err, sol, approach_err)
                    if (pos_err <= pos_tol and axis_err <= axis_tol_deg
                            and approach_err <= axis_tol_deg):
                        break
            finally:
                for j, v in zip(movable_ids, current):
                    p.resetJointState(robot_id, j, float(v),
                                      physicsClientId=self.client)

            if (best is None or best[0] > pos_tol or best[1] > axis_tol_deg
                    or best[3] > axis_tol_deg):
                pos_err, axis_err, sol, approach_err = best
                # Store the failure point: where the best attempt landed.
                # The approach error catches the 180-degrees-about-the-axis
                # solution the y-axis test alone cannot see: fingers pointing
                # UP (backhand) instead of down along the descent.
                stats["failures"].append({
                    "center": np.round(center, 3).tolist(),
                    "ee_target": np.round(ee_target, 3).tolist(),
                    "pos_err": round(float(pos_err), 4),
                    "axis_err_deg": round(float(axis_err), 1),
                    "approach_err_deg": round(float(approach_err), 1),
                    "arm": np.round(sol, 2).tolist(),
                })
                reason = (f"pos_err={pos_err:.3f}m axis_err={axis_err:.1f}deg "
                          f"approach_err={approach_err:.1f}deg "
                          f"at center {np.round(center, 3)}")
                if pos_err > pos_tol:
                    stats["ik_fail"] += 1  # never got the wrist there
                else:
                    stats["fk_fail"] += 1  # there, but jaws misaligned
                if len(stats["reasons"]) < 5:
                    stats["reasons"].append(reason)
                continue

            arm_sol = best[2]

            result = {
                "p1": p1, "p2": p2,
                "grasp_center": center,
                "ee_quat": quat,
                "ee_target": ee_target,
                "arm_joint_ids": arm_ids,
                "joint_angles": arm_sol,
                "score": float(score),
                "strategy": strategy,
            }
            if strategy == "greedy_first":
                return result
            # min-effort tiebreak: stability + distance from current arm pose
            result["_effort"] = result["score"] + joint_damping * np.linalg.norm(arm_sol - current[arm_pos])
            passing.append(result)

        if not passing:
            return None
        return min(passing, key=lambda r: r["_effort"])

    def _grasp_axis(self, candidate):
        p1, p2 = np.asarray(candidate[0], float), np.asarray(candidate[1], float)
        axis = p2 - p1
        return axis / max(np.linalg.norm(axis), 1e-12)

    def _movable_and_arm_ids(self, robot_id):
        movable, arm = [], []
        for j in range(p.getNumJoints(robot_id, physicsClientId=self.client)):
            info = p.getJointInfo(robot_id, j, physicsClientId=self.client)
            if info[2] not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                continue
            movable.append(j)
            if info[1].startswith(ARM_NAME_PREFIXES):
                arm.append(j)
        return movable, arm

    def _fk_verify(self, robot_id, ee_link_index, movable_ids,
                   arm_ids, arm_pos, current, arm_sol,
                   target_pos, target_quat, grasp_axis, descent):
        """IK always returns numbers; command the solution and measure the truth.

        Returns (pos_err_m, axis_err_deg, approach_err_deg) measured at the
        commanded pose. Both rotation errors are needed: the y-axis test is
        sign-symmetric, so a wrist rolled 180 deg about the grasp axis --
        fingers pointing UP, pads above the pads' target band -- still has
        y parallel to the contact line. The x-axis (finger/pad direction)
        must point along the descent.
        """
        try:
            for j, v in zip(movable_ids, current):
                p.resetJointState(robot_id, j, float(v), physicsClientId=self.client)
            for j, v in zip(arm_ids, arm_sol):
                p.resetJointState(robot_id, j, float(v), physicsClientId=self.client)
            state = p.getLinkState(
                robot_id, ee_link_index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )
            # With computeForwardKinematics the computed pose is at [4:6];
            # [0:2] is the stale pose from the last stepSimulation.
            pos, orn = state[4], state[5]
        finally:
            for j, v in zip(movable_ids, current):
                p.resetJointState(robot_id, j, float(v), physicsClientId=self.client)

        pos_err = float(np.linalg.norm(np.asarray(pos) - target_pos))
        rot = _quat_to_mat(np.asarray(orn, dtype=float))
        achieved_y = rot[:, 1]
        # The jaws close along y: it must span the contact line (sign-free,
        # the pair is symmetric).
        cos_y = abs(float(achieved_y @ grasp_axis))
        axis_err = float(np.degrees(np.arccos(min(max(cos_y, -1.0), 1.0))))
        # The pads extend along +x, toward the object: SIGNED comparison --
        # fingers-up (x = -descent) is exactly the roll-180 failure.
        cos_x = float(rot[:, 0] @ descent)
        approach_err = float(np.degrees(np.arccos(min(max(cos_x, -1.0), 1.0))))
        return pos_err, axis_err, approach_err
