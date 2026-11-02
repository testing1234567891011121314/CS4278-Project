"""
Antipodal Grasp Planner for Fetch robot and YCB objects in PyBullet.

  1. sample_antipodal_grasps: vectorized antipodal pair search on the caller's
     point cloud (friction cone + antiparallel normals + point-plane distance),
     geometric composite scoring, greedy center+axis diversity filter.
     Returns best-first tuples (p1, p2, center, quaternion, score) -- score is
     lower-is-better, so candidates[0] is the best geometric grasp.
  2. select_optimal_grasp: IK validation with everything but the 7 arm joints
     locked, FK-verified (calculateInverseKinematics never reports failure),
     axis-only orientation tolerance (parallel-jaw roll about the grasp axis is
     irrelevant), two strategies: "greedy_first" and "score_all".
     Returns the best reachable grasp dict, or None (never raises).
     After each call, self.ik_stats reports calls/ik_fail/fk_fail per strategy.

Grasp convention: the gripper closes along the gripper_link x-axis, so the
contact orientation quaternion maps x onto the grasp axis (p2-p1).
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
                 mu=0.5, ee_forward_offset=0.10):
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
                                min_axis_sep_deg=30.0):
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

        diff = points[:, None, :] - points[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        safe_dist = np.maximum(dist, 1e-12)

        valid = (
            (dist > min_width)
            & (dist <= min(self.gripper_max_width, np.nanmax(ext) + plane_tol))
        )
        if not valid.any():
            return []

        axis = diff / safe_dist[:, :, None]

        # Antiparallel normals: n1 . n2 <= -cos(tol)
        n_dot = normals @ normals.T
        valid &= n_dot <= -np.cos(np.radians(normal_tol_deg))

        # Friction cone at both contacts: the contact normal must face the
        # opposing finger, i.e. -n1 within atan(mu) of the p1->p2 direction.
        cone_cos = np.cos(np.arctan(self.mu))
        n1_dir = (normals[:, None, :] * axis).sum(axis=2)    #  n1 . (p2-p1)/d
        n2_dir = -(normals[None, :, :] * axis).sum(axis=2)   # -n2 . (p2-p1)/d
        valid &= (n1_dir >= cone_cos) & (n2_dir >= cone_cos)

        # Point-plane contact: p2 lies on the tangent plane at p1 (and vice
        # versa) -- |n1 . (p2-p1)| must equal the full separation.
        n1_gap = np.abs((normals[:, None, :] * diff).sum(axis=2))
        n2_gap = np.abs((normals[None, :, :] * diff).sum(axis=2))
        valid &= (np.abs(n1_gap - dist) < plane_tol) & (np.abs(n2_gap - dist) < plane_tol)

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
        #  topdown - |axis . z| near 1 resists gravity without friction
        #  width   - prefer mid-opening: closing margin on both sides
        to_com = center - com
        horizontal = to_com - (to_com * grasp_axis).sum(axis=1, keepdims=True) * grasp_axis
        lever = np.linalg.norm(horizontal, axis=1) / max(float(np.linalg.norm(ext)), 1e-6)
        topdown = 1.0 - np.abs(grasp_axis[:, 2])
        width = 2.0 * np.abs(sep / self.gripper_max_width - 0.5)
        score = 1.0 * lever + 0.8 * topdown + 0.4 * width

        order = np.argsort(score)
        quats = self._contact_orientations(grasp_axis[order])

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

    def _contact_orientations(self, axes):
        """Quaternions whose x-axis is each grasp axis (roll about x arbitrary)."""
        z_ref = np.array([0.0, 0.0, -1.0])
        flip = np.abs(axes @ z_ref) > 0.99
        ref = np.where(flip[:, None], np.tile(np.array([1.0, 0.0, 0.0]), (len(axes), 1)), z_ref)

        z_axis = ref - (ref * axes).sum(axis=1, keepdims=True) * axes
        z_axis /= np.linalg.norm(z_axis, axis=1, keepdims=True)
        y_axis = np.cross(z_axis, axes)
        y_axis /= np.linalg.norm(y_axis, axis=1, keepdims=True)
        z_axis = np.cross(axes, y_axis)

        mats = np.stack([axes, y_axis, z_axis], axis=2)
        return _mat_to_quat(mats)

    # ------------------------------------------------------------------
    # 2. IK validation + selection
    # ------------------------------------------------------------------

    def select_optimal_grasp(self, robot_id, ee_link_index, candidates,
                             strategy="score_all", max_candidates=25,
                             pos_tol=0.01, axis_tol_deg=15.0,
                             min_center_sep=0.03, min_axis_sep_deg=30.0,
                             joint_damping=0.05):
        """Return the best IK-verified grasp dict, or None.

        candidates: tuples (p1, p2, center, quat, score) from
        sample_antipodal_grasps. IK runs over every movable DOF (PyBullet
        returns one entry per movable joint regardless of the limit list),
        with everything but the 7 arm joints pinned at their current pose via
        lower==upper==current. Only the arm entries are extracted; we never
        slice sol[:7]. joint_angles in the result are the 7 arm values.
        """
        if strategy not in ("greedy_first", "score_all"):
            raise ValueError("strategy must be 'greedy_first' or 'score_all'")
        if not candidates:
            self.ik_stats[strategy] = {"calls": 0, "ik_fail": 0, "fk_fail": 0}
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

        # Pin everything; then unlock only the arm joints with their URDF
        # limits. Sentinel/garbage limits (hi < lo) stay pinned at current.
        lower = current.copy()
        upper = current.copy()
        for k, j in enumerate(arm_ids):
            info = p.getJointInfo(robot_id, j, physicsClientId=self.client)
            lo, hi = float(info[8]), float(info[9])
            if hi >= lo:
                lower[arm_pos[k]], upper[arm_pos[k]] = lo, hi

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

        stats = {"calls": 0, "ik_fail": 0, "fk_fail": 0, "reasons": []}
        self.ik_stats[strategy] = stats
        passing = []

        def reject(reason):
            if len(stats["reasons"]) < 5:
                stats["reasons"].append(reason)

        for p1, p2, center, quat, score in picked:
            center = np.asarray(center, dtype=float)
            axis = self._grasp_axis((p1, p2, center, quat, score))
            # p1/p2 ordering makes the axis sign arbitrary; a parallel jaw is
            # symmetric so orient it upward and put the gripper ABOVE the
            # contacts (finger pads forward of gripper_link along +x).
            if axis[2] < 0:
                axis = -axis
            quat = self._contact_orientations(axis[None, :])[0]
            ee_target = center + self.ee_forward_offset * axis

            sol = p.calculateInverseKinematics(
                robot_id, ee_link_index,
                targetPosition=ee_target.tolist(),
                targetOrientation=quat.tolist(),
                lowerLimits=lower.tolist(),
                upperLimits=upper.tolist(),
                jointRanges=np.maximum(upper - lower, 1e-8).tolist(),
                restPoses=current.tolist(),
                maxNumIterations=200,
                residualThreshold=1e-4,
                physicsClientId=self.client,
            )
            stats["calls"] += 1
            sol = np.asarray(sol, dtype=float)

            if sol.shape != lower.shape:
                stats["ik_fail"] += 1
                reject(f"shape {sol.shape} != ({len(movable_ids)},) movable={movable_ids}")
                continue
            if not np.all(np.isfinite(sol)):
                stats["ik_fail"] += 1
                reject("non-finite solution")
                continue

            arm_sol = sol[arm_pos]
            outside = (arm_sol < lower[arm_pos] - 1e-6) | (arm_sol > upper[arm_pos] + 1e-6)
            if np.any(outside):
                idx = int(np.flatnonzero(outside)[0])
                stats["ik_fail"] += 1
                reject(f"arm joint {arm_ids[idx]}={arm_sol[idx]:.4f} outside "
                       f"[{lower[arm_pos][idx]:.4f}, {upper[arm_pos][idx]:.4f}]")
                continue
            if np.max(np.abs(np.delete(sol, arm_pos) - np.delete(current, arm_pos))) > 1e-3:
                stats["ik_fail"] += 1
                reject("pinned joints drifted during IK")
                continue

            if not self._fk_verify(robot_id, ee_link_index, movable_ids,
                                   arm_ids, arm_pos, current, arm_sol,
                                   ee_target, quat, axis, pos_tol, axis_tol_deg):
                stats["fk_fail"] += 1
                continue

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
                   target_pos, target_quat, grasp_axis,
                   pos_tol, axis_tol_deg):
        """IK always returns numbers; command the solution and measure the truth."""
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

        pos_err = np.linalg.norm(np.asarray(pos) - target_pos)
        if pos_err > pos_tol:
            return False
        achieved_x = _quat_to_mat(np.asarray(orn, dtype=float))[:, 0]
        # Signed: +x must face the grasp axis, not -x (pads would face away).
        cos_err = float(achieved_x @ grasp_axis)
        return np.degrees(np.arccos(min(max(cos_err, -1.0), 1.0))) <= axis_tol_deg
