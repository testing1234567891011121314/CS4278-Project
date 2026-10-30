import numpy as np
import pybullet as p


class NavigateToGrasp:
    """
    Plans the mobile-base part of grasping.

    It does not execute IK or close the gripper.
    Its job is to:
        1. select an antipodal grasp,
        2. measure the grasp relative to the robot base,
        3. decide whether the base must move,
        4. provide a free navigation target.
    """

    def __init__(
        self,
        env,
        grasp_planner,
        min_reach=0.25,
        max_reach=0.90,
        min_z=0.20,
        max_z=1.30,
        max_lateral=0.60,
    ):
        self.env = env
        self.grasp_planner = grasp_planner

        self.min_reach = min_reach
        self.max_reach = max_reach
        self.min_z = min_z
        self.max_z = max_z
        self.max_lateral = max_lateral

        self.grasp = None
        self.navigation_goal = None

    def select_grasp(self, obs):
        """
        Select the best geometric antipodal grasp.

        IK is intentionally not checked here because the robot may
        still need to navigate into the arm workspace.
        """
        candidates = self.grasp_planner.sample_antipodal_grasps(
            self.env.object_id,
            points=obs["object_pc"],
            normals=obs["object_normals"],
        )
        if len(candidates) == 0:
            raise RuntimeError("No antipodal grasp candidates found.")

        p1, p2, grasp_center, ee_quat, score = candidates[0]

        self.grasp = {
            "p1": p1,
            "p2": p2,
            "position_world": np.asarray(grasp_center, dtype=float),
            "orientation_world": np.asarray(ee_quat, dtype=float),
            "score": score,
        }

        return self.grasp

    def get_base_pose(self, obs):
        """
        Return the mobile-base pose using the environment's qpos convention.
        """
        qpos = np.asarray(obs["qpos"], dtype=float)

        base_position = np.array([
            qpos[0] - 2.0,
            qpos[1],
            0.0,
        ])

        base_yaw = qpos[2]

        return base_position, base_yaw

    def world_to_base(self, position_world, base_position, base_yaw):
        """
        Transform a world-frame position into the robot-base frame.
        """
        delta = np.asarray(position_world) - base_position

        c = np.cos(base_yaw)
        s = np.sin(base_yaw)

        world_to_base_rotation = np.array([
            [c, s, 0.0],
            [-s, c, 0.0],
            [0.0, 0.0, 1.0],
        ])

        return world_to_base_rotation @ delta

    def relative_grasp_pose(self, obs):
        """
        Return the grasp position and orientation relative to the base.
        """
        if self.grasp is None:
            self.select_grasp(obs)

        base_position, base_yaw = self.get_base_pose(obs)

        grasp_position_base = self.world_to_base(
            self.grasp["position_world"],
            base_position,
            base_yaw,
        )

        base_quaternion_world = p.getQuaternionFromEuler(
            [0.0, 0.0, base_yaw]
        )

        base_inverse = p.invertTransform(
            [0.0, 0.0, 0.0],
            base_quaternion_world,
        )[1]

        grasp_orientation_base = p.multiplyTransforms(
            [0.0, 0.0, 0.0],
            base_inverse,
            [0.0, 0.0, 0.0],
            self.grasp["orientation_world"],
        )[1]

        return grasp_position_base, np.asarray(grasp_orientation_base)

    def classify_reachability(self, grasp_position_base):
        """
        Decide whether the arm can nominally reach the grasp.
        """
        x, y, z = grasp_position_base
        distance = np.linalg.norm(grasp_position_base)

        if distance > self.max_reach:
            return "MOVE_CLOSER"

        if distance < self.min_reach:
            return "MOVE_FARTHER"

        if not self.min_z <= z <= self.max_z:
            return "CHANGE_BASE_POSE"

        if abs(y) > self.max_lateral:
            return "CHANGE_BASE_POSE"

        return "WITHIN_REACH"

    def nearest_free_goal(self, object_xy, occupancy, convert_to_grid,
                          convert_to_world):
        """
        Find the nearest free navigation cell to the object.

        This prevents A* from targeting the table/object obstacle itself.
        """
        object_cell = np.asarray(
            convert_to_grid(object_xy),
            dtype=int,
        )

        free_cells = np.argwhere(occupancy == 0)

        if len(free_cells) == 0:
            raise RuntimeError("The occupancy grid has no free cells.")

        distances = np.linalg.norm(
            free_cells - object_cell,
            axis=1,
        )

        nearest_cell = free_cells[np.argmin(distances)]

        return convert_to_world(tuple(nearest_cell))

    def update(
        self,
        obs,
        occupancy,
        convert_to_grid,
        convert_to_world,
    ):
        """
        Decide whether to navigate or attempt grasping.
        """

        object_pc = np.asarray(obs["object_pc"], dtype=float)

        # ---------------------------------------------------------
        # Generate a geometric grasp only once per base position.
        # Do not perform IK here.
        # ---------------------------------------------------------
        if self.grasp is None:
            candidates = self.grasp_planner.sample_antipodal_grasps(
                self.env.object_id,
                num_samples=500,
            )

            if len(candidates) > 0:
                p1, p2, grasp_center, ee_quat, score = candidates[0]

                self.grasp = {
                    "p1": p1,
                    "p2": p2,
                    "position_world": np.asarray(
                        grasp_center,
                        dtype=float,
                    ),
                    "orientation_world": np.asarray(
                        ee_quat,
                        dtype=float,
                    ),
                    "score": score,
                }

        # ---------------------------------------------------------
        # Case 1: no grasp candidate currently exists.
        # Navigate toward the point cloud.
        # ---------------------------------------------------------
        if self.grasp is None:
            point_cloud_center = np.mean(object_pc, axis=0)

            navigation_goal = self.nearest_free_goal(
                point_cloud_center[:2],
                occupancy,
                convert_to_grid,
                convert_to_world,
            )

            return {
                "state": "NAVIGATE_TO_OBJECT",
                "navigation_goal": navigation_goal,
                "grasp_position_world": None,
                "grasp_orientation_world": None,
            }

        # ---------------------------------------------------------
        # Case 2: a grasp exists. Check whether it is reachable.
        # ---------------------------------------------------------
        grasp_position_base, grasp_orientation_base = (
            self.relative_grasp_pose(obs)
        )

        reach_state = self.classify_reachability(
            grasp_position_base
        )

        if reach_state != "WITHIN_REACH":
            navigation_goal = self.nearest_free_goal(
                self.grasp["position_world"][:2],
                occupancy,
                convert_to_grid,
                convert_to_world,
            )

            return {
                "state": "NAVIGATE_TO_GRASP",
                "navigation_goal": navigation_goal,
                "grasp_position_base": grasp_position_base,
                "grasp_orientation_base": grasp_orientation_base,
                "grasp_position_world": self.grasp["position_world"],
                "grasp_orientation_world": self.grasp["orientation_world"],
            }

        # ---------------------------------------------------------
        # Case 3: the grasp is within the arm workspace.
        # ---------------------------------------------------------
        return {
            "state": "GRASP",
            "navigation_goal": None,
            "grasp_position_base": grasp_position_base,
            "grasp_orientation_base": grasp_orientation_base,
            "grasp_position_world": self.grasp["position_world"],
            "grasp_orientation_world": self.grasp["orientation_world"],
        }