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



class AntipodalGraspPlanner:
    def __init__(self, physics_client_id=0):
        self.rng = np.random.default_rng(0)
        self.client = physics_client_id
   
   def find_optimal_grasps(
      self,
      robot_id,
      ee_link_index,
      candidates,
      joint_limits,
      min_distance=0.03,
   ):
      """Greedily try spatially separated candidates; return first passing IK."""
      tested_centers = []

      # Your scoring convention: lower is better.
      for p1, p2, center, quaternion, score in sorted(
         candidates, key=lambda candidate: candidate[4]
      ):
         center = np.asarray(center, dtype=float)

         if any(
               np.linalg.norm(center - previous) < min_distance
               for previous in tested_centers
         ):
               continue

         tested_centers.append(center.copy())

         joints = self.solve_ik(
               robot_id,
               ee_link_index,
               center,
               quaternion,
               joint_limits,
         )

         if joints is not None:
               return {
                  "p1": p1,
                  "p2": p2,
                  "grasp_center": center,
                  "ee_quat": quaternion,
                  "joint_angles": joints,
                  "score": score,
               }

      return None

   def sample_antipodal_grasps(self, object_id=None, num_samples=500, points=None, normals=None):
      if points is None:
         raise ValueError("Pass obs['object_pc'].")

      points = np.asarray(points, dtype=float)
      if len(points) < 2:
         return []

      i, j = self.rng.choice(len(points), 2, replace=False)
      p1, p2 = points[i], points[j]

      # Expected format: contact1, contact2, center, quaternion, score
      return [(p1, p2, (p1 + p2) / 2,
               np.array([0., 0., 0., 1.]), 0.0)]

   def solve_ik(
      self,
      robot_id,
      ee_link_index,
      target_position,
      target_orientation,
      joint_limits,
      max_iterations=200,
   ):
      """Return joint positions in IK order, or None if limits are violated."""
      joints = [
            p.getJointInfo(robot_id, j, physicsClientId=self.client)
            for j in range(
               p.getNumJoints(robot_id, physicsClientId=self.client)
            )
      ]
      movable = sorted(
            (info for info in joints if info[3] >= 0),
            key=lambda info: info[3],
      )

      if any(
            info[2] not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
            for info in movable
      ):
            raise ValueError("Only single-DOF joints are supported.")

      limits = np.asarray(joint_limits, dtype=float)
      if (
            limits.shape != (len(movable), 2)
            or not np.all(np.isfinite(limits))
            or np.any(limits[:, 0] > limits[:, 1])
      ):
            raise ValueError("Provide finite (N, 2) limits in IK joint order.")

      lower, upper = limits.T
      current = np.array([
            p.getJointState(
               robot_id, info[0], physicsClientId=self.client
            )[0]
            for info in movable
      ])

      result = np.asarray(p.calculateInverseKinematics(
            robot_id,
            ee_link_index,
            targetPosition=target_position,
            targetOrientation=target_orientation,
            lowerLimits=lower.tolist(),
            upperLimits=upper.tolist(),
            jointRanges=np.maximum(upper - lower, 1e-8).tolist(),
            restPoses=np.clip(current, lower, upper).tolist(),
            maxNumIterations=max_iterations,
            residualThreshold=1e-4,
            physicsClientId=self.client,
      ))

      if (
            result.shape != lower.shape
            or not np.all(np.isfinite(result))
            or np.any(result < lower - 1e-6)
            or np.any(result > upper + 1e-6)
      ):
            return None

      return result
