"""
Antipodal Grasp Planner for Fetch robot and YCB objects in PyBullet.

Pipeline (see "Fixes for Antipodal Grasp Generation" doc for full rationale):
  1. Sample a point cloud + normals from the object surface, including top/bottom
     caps that a purely radial ray pattern would miss.
  2. Find antipodal candidate pairs (vectorized), respecting gripper width and the
     friction cone.
  3. Score each candidate by gravity/wrench-space stability (mass + true center of
     mass), not just height -- a grasp near the geometric centroid can still fail if
     the object's real COM is far off-center.
  4. Filter candidates by IK reachability, cheapest-ranked first, and return the
     first grasp that is both stable and reachable: the "optimal" grasp under this
     pipeline's combined quality + feasibility ordering.
  5. Expose the chosen grasp's contact orientations as quaternions relative to the
     robot's vertical (z) axis, decomposed into swing/twist, for base navigation:
     twist -> target base heading; swing -> how much tilt only the arm can resolve.
  6. Provide a per-step grip-force update for use during the lift, to react to the
     object's own gravity + the arm's instantaneous acceleration rather than a
     single, fixed grip force chosen a priori.
"""

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R


class AntipodalGraspPlanner:
    def __init__(
        self,
        gripper_max_width=0.10,
        friction_angle_deg=30.0,
        base_z_axis=(0.0, 0.0, 1.0),
        physics_client_id=0,
    ):
        if not np.isfinite(gripper_max_width) or gripper_max_width <= 0:
            raise ValueError("gripper_max_width must be positive.")

        if not 0.0 < friction_angle_deg < 90.0:
            raise ValueError("friction_angle_deg must lie between 0 and 90.")

        axis = np.asarray(base_z_axis, dtype=float)
        if (
            axis.shape != (3,)
            or not np.all(np.isfinite(axis))
            or np.linalg.norm(axis) < 1e-12
        ):
            raise ValueError("base_z_axis must be a finite nonzero 3D vector.")

        self.physics_client_id = physics_client_id
        self.gripper_max_width = float(gripper_max_width)
        self.friction_angle_deg = float(friction_angle_deg)
        self.min_cos_friction = np.cos(np.radians(friction_angle_deg))
        self.mu = np.tan(np.radians(friction_angle_deg))
        self.base_z_axis = axis / np.linalg.norm(axis)

        self.com = None
        self.mass = None
        self.contact_centroid = None
        self.grasp_axis = None

    # ------------------------------------------------------------------
    # 1. Point cloud sampling 
    # ------------------------------------------------------------------

    def get_object_point_cloud(self, object_id, num_samples=500, cap_grid_n=8):
        client = self.physics_client_id

        lower, upper = p.getAABB(object_id, physicsClientId=client)
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)

        center = (lower + upper) / 2.0
        extent = upper - lower
        margin = 0.02

        radius = np.linalg.norm(extent[:2]) / 2.0 + margin
        angles = np.linspace(
            0.0, 2.0 * np.pi, max(4, int(num_samples) // 4),
            endpoint=False,
        )
        heights = lower[2] + extent[2] * np.array([0.1, 0.35, 0.65, 0.9])

        starts, ends = [], []

        for z in heights:
            for theta in angles:
                starts.append([
                    center[0] + radius * np.cos(theta),
                    center[1] + radius * np.sin(theta),
                    z,
                ])
                ends.append([center[0], center[1], z])

        fractions = np.linspace(0.05, 0.95, max(2, int(cap_grid_n)))
        for fx in fractions:
            for fy in fractions:
                x = lower[0] + fx * extent[0]
                y = lower[1] + fy * extent[1]

                starts.extend([
                    [x, y, upper[2] + margin],
                    [x, y, lower[2] - margin],
                ])
                ends.extend([
                    [x, y, lower[2]],
                    [x, y, upper[2]],
                ])

        points, normals = [], []

        for start in range(0, len(starts), 256):
            results = p.rayTestBatch(
                starts[start:start + 256],
                ends[start:start + 256],
                physicsClientId=client,
            )

            for hit in results:
                if hit[0] != object_id:
                    continue

                point = np.asarray(hit[3], dtype=float)
                normal = np.asarray(hit[4], dtype=float)
                length = np.linalg.norm(normal)

                if (
                    np.all(np.isfinite(point))
                    and np.all(np.isfinite(normal))
                    and length > 1e-12
                ):
                    points.append(point)
                    normals.append(normal / length)

        if not points:
            return np.empty((0, 3)), np.empty((0, 3))

        points = np.asarray(points)
        normals = np.asarray(normals)

        # Remove repeated surface samples.
        _, indices = np.unique(
            np.round(points, decimals=6),
            axis=0,
            return_index=True,
        )
        indices = np.sort(indices)
        return points[indices], normals[indices]

    # ------------------------------------------------------------------
    # 2. Vectorized antipodal candidate search
    # ------------------------------------------------------------------

    def find_antipodal_pairs(self, points, normals):
        points = np.asarray(points, dtype=float)
        normals = np.asarray(normals, dtype=float)

        if (
            points.ndim != 2
            or points.shape[1] != 3
            or normals.shape != points.shape
        ):
            raise ValueError("points and normals must have matching (N, 3) shapes.")

        if not np.all(np.isfinite(points)) or not np.all(np.isfinite(normals)):
            raise ValueError("points and normals must be finite.")

        if len(points) < 2:
            return np.empty((0, 2), dtype=int)

        lengths = np.linalg.norm(normals, axis=1)
        valid_normals = lengths > 1e-12
        unit_normals = normals / np.maximum(lengths[:, None], 1e-12)

        differences = points[:, None, :] - points[None, :, :]
        distances = np.linalg.norm(differences, axis=-1)
        directions = differences / np.maximum(distances[..., None], 1e-12)

        # points[i] - points[j] points outward at contact i.
        cos_i = np.einsum("ijk,ik->ij", directions, unit_normals)
        cos_j = np.einsum("ijk,jk->ij", -directions, unit_normals)

        valid = (
            (distances > 0.01)
            & (distances < self.gripper_max_width)
            & valid_normals[:, None]
            & valid_normals[None, :]
            & (cos_i >= self.min_cos_friction)
            & (cos_j >= self.min_cos_friction)
        )

        return np.argwhere(np.triu(valid, k=1))

    # ------------------------------------------------------------------
    # 3. Gravity / wrench-space stability scoring
    # ------------------------------------------------------------------

    def _object_mass_and_com(self, object_id):
        client = self.physics_client_id

        if p.getNumJoints(object_id, physicsClientId=client) != 0:
            raise ValueError(
                "This COM calculation supports single rigid-body objects only."
            )

        mass = float(
            p.getDynamicsInfo(object_id, -1, physicsClientId=client)[0]
        )
        com, _ = p.getBasePositionAndOrientation(
            object_id,
            physicsClientId=client,
        )

        return mass, np.asarray(com, dtype=float)

    def gravity_wrench(self, object_id, p1, p2):
        """
        Gravity force/torque about the contact centroid, using the object's TRUE
        center of mass (not the geometric centroid of the two contact points).
        """
        mass, com = self._object_mass_and_com(object_id)
        contact_centroid = (p1 + p2) / 2.0
        r = com - contact_centroid
        force = np.array([0.0, 0.0, -mass * 9.81])
        torque = np.cross(r, force)
        return force, torque, mass, com

    def grasp_stability_score(self, object_id, p1, p2, n1, n2):
        """
        Cheap stability proxy: compares the torque gravity exerts about the
        contact centroid (from the lever arm to the TRUE center of mass) against
        the maximum frictional torque the two contacts can plausibly resist.
        Lower required/available ratio = more stable. This is a fast stand-in for
        a full Ferrari-Canny wrench-space solve -- good enough to reject grasps
        whose contact points are far from the object's real center of mass.
        """
        force, torque, mass, com = self.gravity_wrench(object_id, p1, p2)
        torque_mag = np.linalg.norm(torque)

        contact_centroid = (p1 + p2) / 2.0
        lever = np.linalg.norm(com - contact_centroid)
        # crude available resisting torque: friction-limited tangential force at
        # each contact, times the lever arm between the two contacts
        max_tangential_force = self.mu * mass * 9.81
        contact_separation = np.linalg.norm(p2 - p1)
        available_torque = max_tangential_force * contact_separation / 2.0

        if available_torque <= 1e-9:
            return np.inf  # degenerate, treat as worst possible
        ratio = torque_mag / available_torque
        return ratio  # lower is better; < 1 roughly means "should hold"

    # ------------------------------------------------------------------
    # 4. Candidate pose construction + ranking
    # ------------------------------------------------------------------

    def _contact_orientation(self, x_axis):
        x_axis = np.asarray(x_axis, dtype=float)
        length = np.linalg.norm(x_axis)

        if (
            x_axis.shape != (3,)
            or not np.all(np.isfinite(x_axis))
            or length < 1e-12
        ):
            raise ValueError("Grasp axis must be a finite nonzero 3D vector.")

        x_axis = x_axis / length

        reference = np.array([0.0, 0.0, -1.0])
        if abs(np.dot(reference, x_axis)) > 0.99:
            reference = np.array([1.0, 0.0, 0.0])

        z_axis = reference - np.dot(reference, x_axis) * x_axis
        z_axis /= np.linalg.norm(z_axis)

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)

        return R.from_matrix(
            np.column_stack((x_axis, y_axis, z_axis))
        ).as_quat()

    def sample_antipodal_grasps(
        self,
        object_id,
        num_samples=500,
        points=None,
        normals=None,
    ):
        if (points is None) != (normals is None):
            raise ValueError("Provide both points and normals, or neither.")

        if points is None:
            points, normals = self.get_object_point_cloud(
                object_id,
                num_samples,
            )

        points = np.asarray(points, dtype=float)
        normals = np.asarray(normals, dtype=float)
        pairs = self.find_antipodal_pairs(points, normals)

        candidates = []

        for i, j in pairs:
            p1, p2 = points[i], points[j]
            center = (p1 + p2) / 2.0
            axis = (p2 - p1) / np.linalg.norm(p2 - p1)
            quaternion = self._contact_orientation(axis)

            # Ranking heuristic only; this does not certify force closure.
            score = self.grasp_stability_score(
                object_id, p1, p2, normals[i], normals[j]
            )

            if np.isfinite(score):
                candidates.append((p1, p2, center, quaternion, score))

        candidates.sort(key=lambda candidate: candidate[4])
        return candidates

    # ------------------------------------------------------------------
    # 5. IK reachability filter -> the "optimal" grasp
    # ------------------------------------------------------------------

    # def _within_limits(self, joint_angles, joint_limits):
    #     angles = np.asarray(joint_angles, dtype=float)
    #     limits = np.asarray(joint_limits, dtype=float)

    #     if angles.ndim != 1 or limits.shape != (len(angles), 2):
    #         return False

    #     if not np.all(np.isfinite(angles)) or np.any(np.isnan(limits)):
    #         return False

    #     lower = limits[:, 0]
    #     upper = limits[:, 1]

    #     if np.any(lower > upper):
    #         return False

    #     tolerance = 1e-8
    #     return bool(np.all(
    #         (angles >= lower - tolerance)
    #         & (angles <= upper + tolerance)
    #     ))

    def _within_limits(self, joint_angles, joint_limits):
        angles = np.asarray(joint_angles, dtype=float)
        limits = np.asarray(joint_limits, dtype=float)

        if angles.ndim != 1 or limits.shape != (len(angles), 2):
            print(
                "[IK] Joint/limit shape mismatch:",
                f"angles={angles.shape}, limits={limits.shape}",
            )
            return False

        if not np.all(np.isfinite(angles)):
            print("[IK] Non-finite joint values:", angles)
            return False

        lower = limits[:, 0]
        upper = limits[:, 1]

        invalid_limits = np.isnan(lower) | np.isnan(upper) | (lower > upper)
        if np.any(invalid_limits):
            print("[IK] Invalid limits at indices:", np.flatnonzero(invalid_limits))
            return False

        violations = np.flatnonzero(
            (angles < lower - 1e-8) | (angles > upper + 1e-8)
        )

        for index in violations:
            print(
                f"[IK] Paired index {index}: "
                f"value={angles[index]:.6f}, "
                f"limits=[{lower[index]:.6f}, {upper[index]:.6f}]"
            )

        return len(violations) == 0

    def solve_ik(self, robot_id, ee_link_index, target_pos, target_orn,
                 joint_limits=None, max_iterations=100, residual_threshold=1e-4):
        joint_angles = p.calculateInverseKinematics(
            robot_id, ee_link_index, target_pos, target_orn,
            maxNumIterations=max_iterations, residualThreshold=residual_threshold,
        )
        if joint_limits is not None and not self._within_limits(joint_angles, joint_limits):
            return None
        return joint_angles

    # def select_optimal_grasp(self, robot_id, ee_link_index, object_id,
    #                           joint_limits=None, num_samples=500,
    #                           max_candidates_to_try=10):
    #     """
    #     Ties the pipeline together: generate candidates, already ranked by
    #     stability; walk the ranking (cheapest-ranked first) and return the first
    #     one that is also IK-reachable. This is the "optimal grasp" under this
    #     pipeline -- best stability score among those the arm can actually reach.
    #     """
    #     candidates = self.sample_antipodal_grasps(object_id, num_samples)

    #     for p1, p2, grasp_center, quat, score in candidates[:max_candidates_to_try]:
    #         joints = self.solve_ik(robot_id, ee_link_index, grasp_center, quat,
    #                                 joint_limits=joint_limits)
    #         if joints is None:
    #             continue

    #         # cache grasp context for the lift-phase grip controller
    #         mass, com = self._object_mass_and_com(object_id)
    #         self.mass = mass
    #         self.com = com
    #         self.contact_centroid = grasp_center
    #         self.grasp_axis = (p2 - p1) / np.linalg.norm(p2 - p1) if p1 is not None else None

    #         return {
    #             "p1": p1, "p2": p2,
    #             "grasp_center": grasp_center,
    #             "ee_quat": quat,
    #             "stability_score": score,
    #             "joint_angles": joints,
    #         }

    #     return None  # nothing in the top candidates was both stable and reachable
    def select_optimal_grasp(
        self,
        robot_id,
        ee_link_index,
        object_id,
        joint_limits=None,
        num_samples=500,
        max_candidates_to_try=10,
    ):
        candidates = self.sample_antipodal_grasps(
            object_id,
            num_samples=num_samples,
        )

        print(f"[Grasp] Geometric candidates: {len(candidates)}")

        if not candidates:
            print("[Grasp] Failed: no geometric antipodal candidates.")
            return None

        tested = candidates[:max_candidates_to_try]

        for index, (p1, p2, center, quat, score) in enumerate(tested):
            print(
                f"[Grasp] Candidate {index + 1}/{len(tested)}: "
                f"position={np.asarray(center)}, score={score}"
            )

            joints = self.solve_ik(
                robot_id=robot_id,
                ee_link_index=ee_link_index,
                target_pos=center,
                target_orn=quat,
                joint_limits=joint_limits,
            )

            if joints is None:
                print("[Grasp] Rejected by solve_ik().")
                continue

            print(f"[Grasp] IK returned {len(joints)} joint values.")

            mass, com = self._object_mass_and_com(object_id)
            self.mass = mass
            self.com = com
            self.contact_centroid = np.asarray(center, dtype=float)
            self.grasp_axis = (
                (p2 - p1) / np.linalg.norm(p2 - p1)
                if p1 is not None and p2 is not None
                else None
            )

            return {
                "p1": p1,
                "p2": p2,
                "grasp_center": center,
                "ee_quat": quat,
                "stability_score": score,
                "joint_angles": joints,
            }

        print(
            f"[Grasp] Failed: all {len(tested)} tested candidates "
            "were rejected by solve_ik()."
        )
        return None

    # ------------------------------------------------------------------
    # 6. Contact orientation relative to the robot's z-axis (for navigation)
    # ------------------------------------------------------------------

    def swing_twist_about_z(self, quat):
        """
        Decomposes a contact/grasp orientation quaternion into:
          twist -- rotation about the robot's vertical (z) axis (what a mobile
                   base can resolve by turning in place)
          swing -- the remaining tilt away from that axis (what only the arm's
                   joints can resolve)
        Returns (twist_angle_rad, swing_angle_rad, twist_quat, swing_quat).
        """
        rot = R.from_quat(quat)
        xyz = np.array(quat[:3])
        w = quat[3]

        # project the vector part onto the z-axis to isolate the twist quaternion
        proj = np.dot(xyz, self.base_z_axis) * self.base_z_axis
        twist_quat = np.array([proj[0], proj[1], proj[2], w])
        norm = np.linalg.norm(twist_quat)
        if norm < 1e-9:
            twist_quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            twist_quat /= norm

        twist_rot = R.from_quat(twist_quat)
        swing_rot = rot * twist_rot.inv()

        twist_angle = twist_rot.magnitude()
        swing_angle = swing_rot.magnitude()
        return twist_angle, swing_angle, twist_quat, swing_rot.as_quat()

    def base_navigation_target(self, quat):
        """
        Given a contact orientation quaternion, returns a suggested base heading
        (yaw, radians, about the robot's z-axis) and the residual swing angle the
        base cannot resolve on its own -- large swing means the base can only get
        the arm close; the tilt itself is the arm's problem.
        """
        twist_angle, swing_angle, twist_quat, _ = self.swing_twist_about_z(quat)
        # sign/direction of twist about z from the twist quaternion
        twist_rot = R.from_quat(twist_quat)
        yaw = twist_rot.as_euler('xyz')[2] if np.allclose(self.base_z_axis, [0, 0, 1]) \
            else twist_angle  # fallback for non-standard z axes
        return {"target_yaw_rad": yaw, "unresolvable_swing_rad": swing_angle}

    def grasp_relative_quaternion_check(self, quat1, quat2):
        """
        q_rel = q1^-1 * q2 should be close to a 180-degree rotation about the
        grasp axis for a clean antipodal pair; large deviation from pi flags a
        shaky candidate even if the raw friction-cone dot products passed.
        """
        r1 = R.from_quat(quat1)
        r2 = R.from_quat(quat2)
        rel = r1.inv() * r2
        angle = rel.magnitude()
        deviation_from_pi = abs(np.pi - angle)
        return angle, deviation_from_pi

    # ------------------------------------------------------------------
    # 7. Per-step grip-force adjustment during lift
    # ------------------------------------------------------------------

    def required_grip_force(self, linear_accel, angular_accel=None):
        """
        Minimum normal grip force to keep the grasp's combined (gravity +
        inertial) wrench inside the friction cone along the cached grasp axis,
        given the arm's current/planned acceleration. Call once per control step
        during the lift -- not once at grasp selection time.
        """
        if self.mass is None or self.grasp_axis is None:
            raise RuntimeError("Call select_optimal_grasp first to cache grasp context.")

        gravity_force = np.array([0.0, 0.0, -self.mass * 9.81])
        inertial_force = -self.mass * np.array(linear_accel)
        total_force = gravity_force + inertial_force

        normal_component = np.dot(total_force, self.grasp_axis)
        tangential = total_force - normal_component * self.grasp_axis
        tangential_mag = np.linalg.norm(tangential)

        # need enough normal force that tangential stays inside the friction cone
        min_normal_for_friction = tangential_mag / self.mu if self.mu > 1e-6 else np.inf
        return max(abs(normal_component), min_normal_for_friction)

    def adjust_grip_during_lift(self, gripper_id, gripper_joint_index,
                                 linear_accel, angular_accel=None,
                                 force_limits=(0.5, 50.0)):
        """
        Computes and applies the required grip force for the current step,
        clamped to the gripper's safe force range.
        """
        needed_force = self.required_grip_force(linear_accel, angular_accel)
        needed_force = float(np.clip(needed_force, force_limits[0], force_limits[1]))
        p.setJointMotorControl2(
            gripper_id, gripper_joint_index, p.TORQUE_CONTROL, force=needed_force
        )
        return needed_force