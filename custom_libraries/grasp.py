"""
Grasp should implement quaternion based inverse kinematics.
Using the point cloud of the object, it should first calculate a few valid grasp.
While lifting it up, it should take into account that the final optimal position of the object at the grasp could be rotated due to torque while lifting the object.
This means the algorithm has to consider each step of the lifting to adjust the gripping strength accordingly.
"""

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

class AntipodalGraspPlanner:
    """
    Antipodal Grasp Planner for Fetch robot and YCB objects in PyBullet.
    Generates valid antipodal grasp poses based on object geometry and point cloud surface normals.
    """
    def __init__(self, gripper_max_width=0.10, friction_angle_deg=30.0):
        self.gripper_max_width = gripper_max_width  # Fetch parallel gripper max opening (m)
        self.min_cos_friction = np.cos(np.radians(friction_angle_deg))

    def get_object_point_cloud(self, object_id, num_samples=500):
        """
        Extracts point cloud and surface normals from object collision/visual mesh in PyBullet.
        """
        pos, rot = p.getBasePositionAndOrientation(object_id)
        rot_mat = np.array(p.getMatrixFromQuaternion(rot)).reshape(3, 3)
        
        # Get AABB bounds of object
        aabb_min, aabb_max = p.getAABB(object_id)
        center = (np.array(aabb_min) + np.array(aabb_max)) / 2.0
        extents = np.array(aabb_max) - np.array(aabb_min)

        # Uniformly sample points around object surface/AABB for robust candidate selection
        points = []
        normals = []
        
        # Rays directed inward towards object center to compute collision surface contacts
        angles = np.linspace(0, 2 * np.pi, num_samples // 4, endpoint=False)
        heights = np.linspace(aabb_min[2] + 0.02, aabb_max[2] - 0.02, 4)
        
        for z in heights:
            for theta in angles:
                ray_start = center + np.array([1.0 * np.cos(theta), 1.0 * np.sin(theta), z - center[2]])
                ray_end = center + np.array([0.0, 0.0, z - center[2]])
                
                res = p.rayTest(ray_start, ray_end)
                if res and res[0][0] == object_id:
                    hit_pos = np.array(res[0][3])
                    hit_normal = np.array(res[0][4])
                    points.append(hit_pos)
                    normals.append(hit_normal)

        return np.array(points), np.array(normals)

    def sample_antipodal_grasps(self, object_id, top_down_bias=0.7):
        """
        Finds candidate antipodal grasp pairs and builds end-effector target poses (pos, quaternion).
        """
        points, normals = self.get_object_point_cloud(object_id)
        if len(points) < 2:
            # Fallback to AABB top-down grasp if ray sampling returns sparse points
            aabb_min, aabb_max = p.getAABB(object_id)
            target_pos = (np.array(aabb_min) + np.array(aabb_max)) / 2.0
            # Pitch down orientation [roll=0, pitch=pi/2, yaw=0] -> [0.707, 0, 0.707, 0]
            target_orn = p.getQuaternionFromEuler([0, np.pi / 2, 0])
            return [(target_pos, target_orn)]

        valid_grasps = []
        n_pts = len(points)

        for i in range(n_pts):
            for j in range(i + 1, n_pts):
                p1, p2 = points[i], points[j]
                n1, n2 = normals[i], normals[j]

                # Check gripper width constraint
                dist = np.linalg.norm(p2 - p1)
                if dist > self.gripper_max_width or dist < 0.01:
                    continue

                d12 = (p2 - p1) / dist  # Baseline direction between contact points

                # Antipodal criteria: normals must oppose each other and align with baseline d12
                cos_n1_d12 = np.dot(n1, d12)
                cos_n2_d12 = np.dot(n2, -d12)

                if cos_n1_d12 > self.min_cos_friction and cos_n2_d12 > self.min_cos_friction:
                    grasp_center = (p1 + p2) / 2.0
                    
                    # Compute end-effector orientation:
                    # x_ee: gripper finger closing direction along d12
                    # z_ee: approach vector (prefer top-down [0, 0, -1] or side vector)
                    x_axis = d12
                    z_axis = np.array([0.0, 0.0, -1.0])  # Default top-down approach vector
                    
                    # Ensure z_axis is orthogonal to x_axis
                    z_axis = z_axis - np.dot(z_axis, x_axis) * x_axis
                    if np.linalg.norm(z_axis) < 1e-3:
                        z_axis = np.array([1.0, 0.0, 0.0])
                    z_axis /= np.linalg.norm(z_axis)
                    
                    y_axis = np.cross(z_axis, x_axis)
                    y_axis /= np.linalg.norm(y_axis)

                    rot_mat = np.column_stack((x_axis, y_axis, z_axis))
                    quat = R.from_matrix(rot_mat).as_quat()  # [x, y, z, w]

                    valid_grasps.append((grasp_center, quat))

        # Sort candidate grasps (prefer top-down approach angles)
        valid_grasps.sort(key=lambda g: -g[0][2])
        return valid_grasps