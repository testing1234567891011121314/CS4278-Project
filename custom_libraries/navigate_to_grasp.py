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
        self.failed_goals = set()

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

    def mark_goal_failed(self, goal_world):
        """Called when the arm proved unable to grasp from a parked pose."""
        self.failed_goals.add(tuple(np.round(np.asarray(goal_world), 3)))

    def nearest_free_goal(self, object_xy, occupancy, convert_to_grid,
                          convert_to_world, cell_size=0.1, robot_xy=None):
        """
        Find a free navigation cell from which the arm can actually reach.

        Prefers cells inside an arm-reach annulus around the object, nearest
        to the robot's current pose, skipping cells previously proven
        un-graspable. Falls back to plain nearest-free when the annulus is
        empty (old behaviour, used when no grasp exists yet).
        """
        object_cell = np.asarray(
            convert_to_grid(object_xy),
            dtype=int,
        )

        free_cells = np.argwhere(occupancy == 0)

        if len(free_cells) == 0:
            raise RuntimeError("The occupancy grid has no free cells.")

        def to_world(cell):
            return tuple(np.round(np.asarray(convert_to_world(tuple(cell))), 3))

        if self.failed_goals:
            ok = np.array([to_world(c) not in self.failed_goals for c in free_cells])
            if not ok.any():
                raise RuntimeError(
                    "Every free cell near the object has been proven "
                    "un-graspable by IK; the arm cannot reach the object "
                    "from anywhere the base can stand."
                )
            free_cells = free_cells[ok]

        dist_to_object = (
            np.linalg.norm(free_cells - object_cell, axis=1) * cell_size
        )
        annulus = (dist_to_object >= self.min_reach) & (dist_to_object <= self.max_reach)
        candidates = free_cells[annulus]

        if len(candidates) == 0:
            print(f"nearest_free_goal: annulus empty; {len(free_cells)} free cells, "
                  f"closest free cell {dist_to_object.min():.2f} m from object "
                  f"-- falling back to all free cells")
            candidates = free_cells

        if robot_xy is not None:
            robot_cell = np.asarray(convert_to_grid(robot_xy), dtype=float)
            best = candidates[np.argmin(np.linalg.norm(candidates - robot_cell, axis=1))]
        else:
            best = candidates[np.argmin(np.linalg.norm(candidates - object_cell, axis=1))]

        return to_world(best)

    def update(
        self,
        obs,
        occupancy,
        convert_to_grid,
        convert_to_world,
        cell_size=0.1,
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
                points=obs["object_pc"],
                normals=obs["object_normals"],
            )
            if len(candidates) == 0:
                print("sampler starved:", self.grasp_planner.sample_stats)

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
                cell_size,
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
            base_position, _ = self.get_base_pose(obs)
            navigation_goal = self.nearest_free_goal(
                self.grasp["position_world"][:2],
                occupancy,
                convert_to_grid,
                convert_to_world,
                cell_size,
                robot_xy=base_position[:2],
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