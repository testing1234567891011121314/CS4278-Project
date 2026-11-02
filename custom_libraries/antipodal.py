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
        sample_antipodal_grasps. Only arm joints move; base, torso, head and
        fingers are locked at their current pose, so their -1 sentinel limits
        never reach PyBullet.
        """
        if strategy not in ("greedy_first", "score_all"):
            raise ValueError("strategy must be 'greedy_first' or 'score_all'")
        if not candidates:
            self.ik_stats[strategy] = {"calls": 0, "ik_fail": 0, "fk_fail": 0}
            return None

        arm_ids, locked_ids = self._arm_joint_ids(robot_id)
        if len(arm_ids) != 7:
            raise ValueError(
                f"Expected 7 arm joints, found {len(arm_ids)}: check robot URDF naming"
            )

        current = np.array([
            p.getJointState(robot_id, j, physicsClientId=self.client)[0]
            for j in arm_ids
        ])

        lower, upper = [], []
        for j in arm_ids:
            info = p.getJointInfo(robot_id, j, physicsClientId=self.client)
            lo, hi = float(info[8]), float(info[9])
            if hi < lo:
                hi = lo
            lower.append(lo)
            upper.append(hi)
        lower = np.array(lower)
        upper = np.array(upper)
        locked_states = [
            (j, p.getJointState(robot_id, j, physicsClientId=self.client)[0])
            for j in locked_ids
        ]

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

        stats = {"calls": 0, "ik_fail": 0, "fk_fail": 0}
        self.ik_stats[strategy] = stats
        passing = []

        for p1, p2, center, quat, score in picked:
            center = np.asarray(center, dtype=float)
            quat = np.asarray(quat, dtype=float)
            axis = self._grasp_axis((p1, p2, center, quat, score))
            ee_target = center + self.ee_forward_offset * axis

            sol = p.calculateInverseKinematics(
                robot_id, ee_link_index,
                targetPosition=ee_target.tolist(),
                targetOrientation=quat.tolist(),
                lowerLimits=lower.tolist(),
                upperLimits=upper.tolist(),
                jointRanges=np.maximum(upper - lower, 1e-8).tolist(),
                restPoses=np.clip(current, lower, upper).tolist(),
                maxNumIterations=200,
                residualThreshold=1e-4,
                physicsClientId=self.client,
            )
            stats["calls"] += 1
            sol = np.asarray(sol, dtype=float)

            if (sol.shape != lower.shape or not np.all(np.isfinite(sol))
                    or np.any(sol < lower - 1e-6) or np.any(sol > upper + 1e-6)):
                stats["ik_fail"] += 1
                continue

            if not self._fk_verify(robot_id, ee_link_index, arm_ids, sol,
                                   ee_target, quat, axis, pos_tol, axis_tol_deg,
                                   locked_states):
                stats["fk_fail"] += 1
                continue

            result = {
                "p1": p1, "p2": p2,
                "grasp_center": center,
                "ee_quat": quat,
                "ee_target": ee_target,
                "arm_joint_ids": arm_ids,
                "joint_angles": sol,
                "score": float(score),
                "strategy": strategy,
            }
            if strategy == "greedy_first":
                return result
            # min-effort tiebreak: stability + distance from current arm pose
            result["_effort"] = result["score"] + joint_damping * np.linalg.norm(sol - current)
            passing.append(result)

        if not passing:
            return None
        return min(passing, key=lambda r: r["_effort"])

    def _grasp_axis(self, candidate):
        p1, p2 = np.asarray(candidate[0], float), np.asarray(candidate[1], float)
        axis = p2 - p1
        return axis / max(np.linalg.norm(axis), 1e-12)

    def _arm_joint_ids(self, robot_id):
        arm, locked = [], []
        for j in range(p.getNumJoints(robot_id, physicsClientId=self.client)):
            jtype = p.getJointInfo(robot_id, j, physicsClientId=self.client)[2]
            if jtype not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                continue
            name = p.getJointInfo(robot_id, j, physicsClientId=self.client)[1]
            (arm if name.startswith(ARM_NAME_PREFIXES) else locked).append(j)
        return arm, locked

    def _fk_verify(self, robot_id, ee_link_index, arm_ids, sol,
                   target_pos, target_quat, grasp_axis,
                   pos_tol, axis_tol_deg, locked_states):
        """IK always returns numbers; command the solution and measure the truth."""
        saved = [p.getJointState(robot_id, j, physicsClientId=self.client)[0] for j in arm_ids]
        try:
            for j, v in locked_states:
                p.resetJointState(robot_id, j, v, physicsClientId=self.client)
            for j, v in zip(arm_ids, sol):
                p.resetJointState(robot_id, j, float(v), physicsClientId=self.client)
            pos, orn = p.getLinkState(
                robot_id, ee_link_index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )[:4][:2]
        finally:
            for j, v in zip(arm_ids, saved):
                p.resetJointState(robot_id, j, v, physicsClientId=self.client)

        if np.linalg.norm(np.asarray(pos) - target_pos) > pos_tol:
            return False
        achieved_x = _quat_to_mat(np.asarray(orn, dtype=float))[:, 0]
        # abs(): fingers closing along +x or -x are the same grasp
        cos_err = abs(float(achieved_x @ grasp_axis))
        return np.degrees(np.arccos(min(cos_err, 1.0))) <= axis_tol_deg
