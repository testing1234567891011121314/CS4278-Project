import numpy as np
import pybullet as p
from utils import closest_joint_values  # Handles joint angle wrapping
from custom_libraries.a_star import a_star
from custom_libraries.curved_navigator import CurvedNavigator 
from custom_libraries.wall_projection import build_wall_collision_grid
from custom_libraries.antipodal import AntipodalGraspPlanner
from custom_libraries.navigate_to_grasp import NavigateToGrasp

class Solver:
    def __init__(self, env):
        self.env = env
        self.robot_radius = self.get_base_radius()
        if not np.isfinite(self.robot_radius) or self.robot_radius <= 0:
            raise ValueError("robot_radius must be a positive number in metres.")
        
        self.action = env.agent_qpos # np.zeros(self.env.action_size)
        self.action_scale = np.array([0.01] * env.action_size)
        self.action_scale[:3] = 0.02  # root movement
        self.action_scale[-2:] = 0.005  # gripper
        self.action_upper_bound = env.joint_upper
        self.action_lower_bound = env.joint_lower
        self.turn = 0
        self.forward = 0
        self.yaw = 0
        self.torso_lift = 0
        self.head_pan = 0
        self.head_tilt = 0
        self.shoulder_pan = 0
        self.shoulder_lift = 0
        self.upperarm_roll = 0
        self.elbow_flex = 0
        self.forearm_roll = 0
        self.wrist_flex = 0
        self.wrist_roll = 0
        self.gripper_open = 0

        self.create_map(env) # stores self.occupancy, self.cell_size, self.x_min, self.y_max
        self.inflated_occupancy = self.create_navigable_map() # returns inflated occupancy grid
        self.curved_nav = CurvedNavigator(self)
        self.path = None
        self.grasp_planner = AntipodalGraspPlanner(
            physics_client_id=env.pb_physics_client,
        )
        self.ee_link_index = self.find_link_index("gripper_link")
        self.navigate_grasp = NavigateToGrasp(
            env,
            self.grasp_planner,
        )
        self.debug_joint_mapping()

    def get_action(self, env):
        obs = env._get_obs()

        if not hasattr(self, "_action_phase"):
            self._action_phase = "NAVIGATE"
            self._active_navigation_goal = None

        # Persist grasp execution across timesteps.
        if self._action_phase == "GRASP":
            self._last_state = "GRASP"
            action = self.grasp(env)
            self.action = np.asarray(action, dtype=float).copy()
            return self.action.copy()

        # Check the existing goal before update() can replace it.
        if (
            self._active_navigation_goal is not None
            and self.at_pregrasp_pose(
                env,
                goal=self._active_navigation_goal,
                obs=obs,
            )
        ):
            self._action_phase = "GRASP"
            self._last_state = "GRASP"

            self.path = None
            self._path_goal = None
            self.curved_nav.reset()

            self.forward = 0.0
            self.turn = 0.0
            self.yaw = 0.0

            print("Navigation complete: attempting grasp.")

            # grasp() selects a candidate and attempts IK on its first call.
            action = self.grasp(env)
            self.action = np.asarray(action, dtype=float).copy()
            return self.action.copy()

        # Obtain a decision only when no navigation goal is active.
        if self._active_navigation_goal is None:
            decision = self.navigate_grasp.update(
                obs,
                self.inflated_occupancy,
                self.convert_to_grid,
                self.convert_to_world,
            )

            state = decision["state"]
            self._last_state = state

            if state == "GRASP":
                self._action_phase = "GRASP"
                action = self.grasp(env)
                self.action = np.asarray(action, dtype=float).copy()
                return self.action.copy()

            if state not in {
                "NAVIGATE_TO_OBJECT",
                "NAVIGATE_TO_GRASP",
            }:
                raise RuntimeError(f"Unhandled planner state: {state}")

            goal = np.asarray(decision["navigation_goal"], dtype=float)

            if goal.shape != (2,) or not np.all(np.isfinite(goal)):
                raise ValueError(f"Invalid navigation goal: {goal}")

            self._active_navigation_goal = goal.copy()

        return self.navigate(
            env,
            self._active_navigation_goal,
        )


    def at_pregrasp_pose(self, env, tolerance=0.10, goal=None, obs=None):
        if obs is None:
            obs = env._get_obs()

        if goal is None:
            goal = getattr(self, "_active_navigation_goal", None)

        if goal is None:
            return False

        qpos = np.asarray(obs["qpos"], dtype=float)
        current_xy = np.array([qpos[0] - 2.0, qpos[1]])

        return bool(
            np.linalg.norm(current_xy - np.asarray(goal)) < tolerance
        )

    def grasp(self, env):
        """
        Plan and execute one grasp.

        The object position and gripper orientation are expressed relative
        to the robot base. Call this once per simulation timestep.
        """

        obs = env._get_obs()
        qpos = np.asarray(obs["qpos"], dtype=float)

        # ---------------------------------------------------------
        # Select and cache a grasp only once
        # ---------------------------------------------------------
        if not hasattr(self, "_grasp_plan"):
            grasp = self.grasp_planner.select_optimal_grasp(
                robot_id=env.robot_id,
                ee_link_index=self.ee_link_index,
                object_id=env.object_id,
                joint_limits=list(zip(env.joint_lower, env.joint_upper)),
            )

            if grasp is None:
                raise RuntimeError("No reachable antipodal grasp found.")

            self._grasp_plan = grasp
            self._grasp_joint_target = np.asarray(
                grasp["joint_angles"][:len(qpos)],
                dtype=float,
            )

        # ---------------------------------------------------------
        # Object position relative to robot base
        # ---------------------------------------------------------
        object_world = np.asarray(obs["object_pos"], dtype=float)

        # qpos[0:2] uses the robot's base coordinates.
        # Keep the same coordinate convention used by navigate().
        base_world = np.array([
            qpos[0] - 2.0,
            qpos[1],
            0.0,
        ])

        base_yaw = qpos[2]

        c = np.cos(base_yaw)
        s = np.sin(base_yaw)

        world_to_base = np.array([
            [ c, s, 0.0],
            [-s, c, 0.0],
            [0.0, 0.0, 1.0],
        ])

        object_base = world_to_base @ (object_world - base_world)

        # ---------------------------------------------------------
        # Grasp orientation relative to robot base
        # ---------------------------------------------------------
        grasp_quat_world = self._grasp_plan["ee_quat"]

        base_quat_world = p.getQuaternionFromEuler(
            [0.0, 0.0, base_yaw]
        )

        base_quat_inverse = p.invertTransform(
            [0.0, 0.0, 0.0],
            base_quat_world,
        )[1]

        grasp_quat_base = p.multiplyTransforms(
            [0.0, 0.0, 0.0],
            base_quat_inverse,
            [0.0, 0.0, 0.0],
            grasp_quat_world,
        )[1]

        # ---------------------------------------------------------
        # Check whether the object is within the arm's workspace
        # ---------------------------------------------------------
        distance = np.linalg.norm(object_base)

        within_reach = (
            0.25 <= object_base[0] <= 0.90
            and abs(object_base[1]) <= 0.60
            and 0.20 <= object_base[2] <= 1.30
            and distance <= 0.95
        )
        # if not within_reach:
        #     # Still navigate toward the object.
        #     return self.navigate(env, object_world[:2])

        # ---------------------------------------------------------
        # Move arm toward the planned grasp
        # ---------------------------------------------------------
        target = qpos.copy()

        arm_start = 6
        arm_end = min(13, len(target))

        target[arm_start:arm_end] = self._grasp_joint_target[
            arm_start:arm_end
        ]

        # Close gripper after the arm has reached the grasp
        arm_error = np.max(
            np.abs(qpos[arm_start:arm_end] - target[arm_start:arm_end])
        )

        if arm_error < 0.05:
            target[-2:] = env.joint_upper[-2:]
        else:
            target[-2:] = qpos[-2:]

        return np.clip(
            target,
            env.joint_lower,
            env.joint_upper,
        )

    def find_link_index(self, name):
        robot_id = self.env.robot_id
        client = self.env.pb_physics_client

        for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
            info = p.getJointInfo(robot_id, i, physicsClientId=client)
            link_name = info[12].decode("utf-8")

            if link_name == name:
                return i

        raise ValueError(f"Link not found: {name}")

    def navigate(self, env, goal_pos):
        obs = env._get_obs()
        qpos = np.asarray(obs["qpos"], dtype=float)

        goal_pos = np.asarray(goal_pos, dtype=float)

        if goal_pos.shape != (2,) or not np.all(np.isfinite(goal_pos)):
            raise ValueError(
                f"Expected a finite navigation goal of shape (2,), "
                f"got {goal_pos}"
            )

        # Prepare the travel posture before moving the base.
        if not getattr(self, "_travel_pose_complete", False):
            pose_action = self.moving_pose(obs)

            if self.moving_pose_ready:
                self._travel_pose_complete = True
                self.inflated_occupancy = self.create_navigable_map()

                self.path = None
                self._path_goal = None
                self.waypoint_index = 0
                self.curved_nav.reset()

            return pose_action

        # Retain your existing joint-to-world coordinate convention.
        base_xy = np.array([qpos[0] - 2.0, qpos[1]])

        start = tuple(self.convert_to_grid(base_xy))
        goal = tuple(self.convert_to_grid(goal_pos))

        rows, cols = self.inflated_occupancy.shape

        def in_bounds(cell):
            row, col = cell
            return 0 <= row < rows and 0 <= col < cols

        if not in_bounds(start):
            raise ValueError(f"Robot cell is outside the map: {start}")

        if not in_bounds(goal):
            raise ValueError(f"Goal cell is outside the map: {goal}")

        if self.inflated_occupancy[goal] != 0:
            raise ValueError(
                f"Navigation goal {goal} is occupied. "
                "Choose a free approach position beside the table."
            )

        # A cached path is valid only for its original goal.
        if goal != getattr(self, "_path_goal", None):
            self.path = None
            self._path_goal = goal
            self.waypoint_index = 0
            self.curved_nav.reset()
            self._last_nav_status = None

        if self.path is None:
            path = a_star(
                start,
                goal,
                self.inflated_occupancy,
            )

            if path is None or len(path) == 0:
                raise RuntimeError(
                    f"No navigation path from {start} to {goal}"
                )

            self.path = path
            self.waypoint_index = 0
            self.curved_nav.reset()

            print("Path length:", len(self.path))

        self.forward = 0.0
        self.turn = 0.0
        self.yaw = 0.0

        # Preserve commanded travel posture; anchor base targets
        # to measured joints so old translation commands do not accumulate.
        self.action = np.asarray(self.action, dtype=float).copy()
        self.action[:3] = qpos[:3]

        step_xy = np.asarray(
            self.curved_nav.step(base_xy, self.path),
            dtype=float,
        )

        if step_xy.shape != (2,) or not np.all(np.isfinite(step_xy)):
            raise RuntimeError(
                f"Navigator returned an invalid XY step: {step_xy}"
            )

        self.forward = step_xy[0] / self.action_scale[0]
        self.turn = step_xy[1] / self.action_scale[1]
        self.waypoint_index = self.curved_nav.grid_index

        status = self.curved_nav.status

        if status != getattr(self, "_last_nav_status", None):
            print("Navigation:", status)
            self._last_nav_status = status

        unit_action = np.array([
            self.forward,
            self.turn,
            self.yaw,
            self.torso_lift,
            self.head_pan,
            self.head_tilt,
            self.shoulder_pan,
            self.shoulder_lift,
            self.upperarm_roll,
            self.elbow_flex,
            self.forearm_roll,
            self.wrist_flex,
            self.wrist_roll,
            self.gripper_open,
            self.gripper_open,
        ], dtype=float)

        self.action = np.clip(
            self.action + self.action_scale * unit_action,
            self.action_lower_bound,
            self.action_upper_bound,
        )

        return self.action.copy()

    def moving_pose(self, obs):
        """Move gradually toward a fixed test pose while holding the base still."""
        qpos = np.asarray(obs["qpos"], dtype=float)

        # Initialise once: preserve the starting base, torso, head and fingers.
        if not hasattr(self, "_moving_pose_target"):
            target = qpos.copy()

            # Illustrative test angles in radians.
            # These need validation on your robot; they are not a verified safe pose.
            target[6] = 0.0     # shoulder_pan
            target[7] = -1.0    # shoulder_lift
            target[8] = 0.0     # upperarm_roll
            target[9] = -2.0    # elbow_flex
            target[10] = 0.0    # forearm_roll
            target[11] = 1.0    # wrist_flex
            target[12] = 0.0    # wrist_roll

            self._moving_pose_target = np.clip(
                target,
                self.action_lower_bound,
                self.action_upper_bound,
            )

        target = self._moving_pose_target
        arm = slice(6, 13)

        # Absolute targets: retain the original base position.
        action = target.copy()

        # Maximum commanded joint increment per action, in radians.
        max_joint_step = 0.05
        error = target[arm] - qpos[arm]
        action[arm] = qpos[arm] + np.clip(
            error, -max_joint_step, max_joint_step
        )

        qvel = np.asarray(obs["qvel"], dtype=float)
        self.moving_pose_ready = (
            np.max(np.abs(error)) < 0.03
            and np.max(np.abs(qvel[arm])) < 0.05
        )

        self.action = np.clip(
            action,
            self.action_lower_bound,
            self.action_upper_bound,
        )
        return self.action.copy()

    ## Helper function to compute the robot's base radius from its collision mesh.
    def get_base_radius(self):
        client = self.env.pb_physics_client
        robot_id = self.env.robot_id

        # 1. Find base_link without hard-coded body or joint IDs.
        link_names = {
            -1: p.getBodyInfo(
                robot_id, physicsClientId=client
            )[0].decode("utf-8")
        }

        for index in range(p.getNumJoints(
            robot_id, physicsClientId=client
        )):
            info = p.getJointInfo(
                robot_id, index, physicsClientId=client
            )
            link_names[index] = info[12].decode("utf-8")

        matches = [
            index for index, name in link_names.items()
            if name == "base_link"
        ]

        if len(matches) != 1:
            raise RuntimeError(
                f"Expected one base_link; found links: {link_names}"
            )

        link_index = matches[0]

        # 2. Check that this link has mesh collision geometry.
        shapes = p.getCollisionShapeData(
            robot_id, link_index, physicsClientId=client
        )

        # This implementation targets the single-mesh Fetch chassis.
        if len(shapes) != 1 or shapes[0][2] != p.GEOM_MESH:
            raise RuntimeError(
                "Expected one collision mesh on base_link."
            )

        # 3. Read simulation vertices, not graphics vertices.
        count, vertices = p.getMeshData(
            robot_id,
            linkIndex=link_index,
            flags=p.MESH_DATA_SIMULATION_MESH,
            physicsClientId=client,
        )

        vertices = np.asarray(vertices, dtype=float)

        if (
            count == 0
            or vertices.shape != (count, 3)
            or not np.all(np.isfinite(vertices))
        ):
            raise RuntimeError(
                "PyBullet did not return valid base collision vertices."
            )

        # 4. Transform rigid-body simulation vertices from the local
        # inertial frame into the base_link frame.
        dynamics = p.getDynamicsInfo(
            robot_id, link_index, physicsClientId=client
        )
        inertial_position = np.asarray(dynamics[3])
        inertial_rotation = np.asarray(
            p.getMatrixFromQuaternion(dynamics[4])
        ).reshape(3, 3)

        vertices_base = (
            vertices @ inertial_rotation.T + inertial_position
        )

        # 5. Sanity-check the assumed vertex frame against world bounds.
        if link_index == -1:
            com_position, com_orientation = (
                p.getBasePositionAndOrientation(
                    robot_id, physicsClientId=client
                )
            )
        else:
            state = p.getLinkState(
                robot_id,
                link_index,
                computeForwardKinematics=True,
                physicsClientId=client,
            )
            com_position, com_orientation = state[0], state[1]

        world_rotation = np.asarray(
            p.getMatrixFromQuaternion(com_orientation)
        ).reshape(3, 3)

        vertices_world = (
            vertices @ world_rotation.T + np.asarray(com_position)
        )

        lower, upper = p.getAABB(
            robot_id, link_index, physicsClientId=client
        )

        tolerance = 1e-3
        if (
            np.any(vertices_world < np.asarray(lower) - tolerance)
            or np.any(vertices_world > np.asarray(upper) + tolerance)
        ):
            raise RuntimeError(
                "Returned mesh vertices do not fit the collision AABB. "
                "Check the mesh coordinate convention for this build."
            )

        # 6. Radius comes from actual returned vertices, NOT AABB corners.
        radius = float(
            np.linalg.norm(vertices_base[:, :2], axis=1).max()
        )

        if not np.isfinite(radius) or radius <= 0:
            raise RuntimeError("Invalid mesh-derived radius.")

        print("Chassis link index:", link_index)
        print("Collision vertex count:", count)
        print("Base-frame minimum XYZ:", vertices_base.min(axis=0))
        print("Base-frame maximum XYZ:", vertices_base.max(axis=0))
        print(f"Mesh-derived base radius: {radius:.6f} m")

        return radius

    ### Helper functions for creating a map of the environment and navigating the environment. ###
    ## When initialising the solver, this function is called to create a map of the environment.
    def create_map(self, env):
        print("Creating map of the environment...")
        obs = env._get_obs()
        cube_positions = obs["cube_positions"]

        wall_size = 1.0
        cell_size = 0.1

        # Map boundaries are half a wall beyond the outermost wall centres.
        x_min = cube_positions[:, 0].min() - wall_size / 2
        x_max = cube_positions[:, 0].max() + wall_size / 2
        y_min = cube_positions[:, 1].min() - wall_size / 2
        y_max = cube_positions[:, 1].max() + wall_size / 2

        n_cols = int(np.ceil((x_max - x_min) / cell_size))
        n_rows = int(np.ceil((y_max - y_min) / cell_size))

        occupancy = np.zeros((n_rows, n_cols), dtype=np.uint8)

        for x, y in cube_positions:
            # Wall boundaries expressed in grid coordinates.
            # Rounding removes floating-point noise because these walls
            # align exactly with the 0.1 m grid.
            col_start = int(round((x - wall_size / 2 - x_min) / cell_size))
            col_end = int(round((x + wall_size / 2 - x_min) / cell_size))

            row_start = int(round((y_max - y - wall_size / 2) / cell_size))
            row_end = int(round((y_max - y + wall_size / 2) / cell_size))

            occupancy[row_start:row_end, col_start:col_end] = 1

        # Store the map and its coordinate reference for later navigation.
        self.cell_size = cell_size
        self.x_min = x_min
        self.y_max = y_max

        # At this point, occupancy contains only maze walls.
        self.wall_occupancy = occupancy.copy()

        # Keep table/additional obstacles separate.
        self.extra_occupancy = np.zeros_like(occupancy)
        extra_bounds = self.get_extra_obstacle_bounds(cube_positions)
        self.mark_obstacle_bounds(self.extra_occupancy, extra_bounds)

        self.occupancy = (
            self.wall_occupancy | self.extra_occupancy
        )

        print("Map created. Occupancy grid : ", occupancy)

    def create_navigable_map(self):
        safety_margin = 0.05

        # Maze: use projected geometry of all robot collision links.
        wall_blocked = build_wall_collision_grid(
            self.env,
            self.wall_occupancy,
            self.cell_size,
            safety_margin=safety_margin,
        )

        # Additional obstacles: retain the existing base-radius treatment
        # temporarily. This is NOT whole-arm table collision checking.
        radius_cells = int(np.ceil(
            (self.robot_radius + safety_margin) / self.cell_size
        ))

        rows, cols = self.extra_occupancy.shape
        padded = np.pad(
            self.extra_occupancy.astype(bool),
            radius_cells,
            mode="constant",
        )
        extra_blocked = np.zeros((rows, cols), dtype=bool)

        for dr in range(2 * radius_cells + 1):
            for dc in range(2 * radius_cells + 1):
                extra_blocked |= padded[
                    dr:dr + rows,
                    dc:dc + cols,
                ]

        return (
            wall_blocked.astype(bool) | extra_blocked
        ).astype(np.uint8)

    def convert_to_grid(self, position):
        col = int(np.floor(
            (position[0] - self.x_min) / self.cell_size
        ))
        row = int(np.floor(
            (self.y_max - position[1]) / self.cell_size
        ))
        return row, col

    def convert_to_world(self, cell):
        row, col = cell
        x = self.x_min + (col + 0.5) * self.cell_size
        y = self.y_max - (row + 0.5) * self.cell_size
        return np.array([x, y])

    ### Helper functions for querying pybullet directly to find the table. ###
    def get_extra_obstacle_bounds(self, cube_positions):
        """Read additional obstacle bounds without changing the simulation."""
        client = self.env.pb_physics_client
        obstacles = []

        excluded = {
            self.env.robot_id,
            self.env.object_id,       # Grasp target: handled separately.
            self.env.goal_marker_id,
        }

        for index in range(p.getNumBodies(physicsClientId=client)):
            body_id = p.getBodyUniqueId(
                index, physicsClientId=client
            )

            if body_id in excluded:
                continue

            link_bounds = []

            for link in range(
                -1,
                p.getNumJoints(body_id, physicsClientId=client),
            ):
                shapes = p.getCollisionShapeData(
                    body_id, link, physicsClientId=client
                )

                if not shapes:
                    continue

                lower, upper = p.getAABB(
                    body_id, link, physicsClientId=client
                )
                link_bounds.append((
                    np.asarray(lower),
                    np.asarray(upper),
                ))

            if not link_bounds:
                continue

            body_min = np.min([b[0] for b in link_bounds], axis=0)
            body_max = np.max([b[1] for b in link_bounds], axis=0)

            # This environment's ground surface is at z = 0.
            if body_max[2] <= 0.01:
                continue

            # Recognise the merged maze by its bounds enclosing every
            # supplied wall centre. Its walls are already mapped separately.
            contains_walls = np.all(
                (cube_positions >= body_min[:2] - 1e-3)
                & (cube_positions <= body_max[:2] + 1e-3)
            )

            if contains_walls:
                continue

            obstacles.extend(link_bounds)

        return obstacles

    def mark_obstacle_bounds(self, occupancy, obstacle_bounds):
        rows, cols = occupancy.shape

        for lower, upper in obstacle_bounds:
            col_start = int(np.floor(
                (lower[0] - self.x_min) / self.cell_size
            ))
            col_end = int(np.ceil(
                (upper[0] - self.x_min) / self.cell_size
            ))

            row_start = int(np.floor(
                (self.y_max - upper[1]) / self.cell_size
            ))
            row_end = int(np.ceil(
                (self.y_max - lower[1]) / self.cell_size
            ))

            col_start = max(0, min(cols, col_start))
            col_end = max(0, min(cols, col_end))
            row_start = max(0, min(rows, row_start))
            row_end = max(0, min(rows, row_end))

            occupancy[row_start:row_end, col_start:col_end] = 1
    def base_clearance(self, position):
        # Cache occupied-cell centres once; includes any mapped table.
        if not hasattr(self, "_obstacle_centres"):
            rows, cols = np.nonzero(self.occupancy)

            self._obstacle_centres = np.column_stack((
                self.x_min + (cols + 0.5) * self.cell_size,
                self.y_max - (rows + 0.5) * self.cell_size,
            ))

        if len(self._obstacle_centres) == 0:
            return float("inf")

        distances = np.linalg.norm(
            self._obstacle_centres - np.asarray(position),
            axis=1,
        )

        # Subtract cell extent and robot radius for conservative clearance.
        half_cell_diagonal = self.cell_size / np.sqrt(2)

        return float(
            distances.min() - half_cell_diagonal - self.robot_radius
        )


    ## Debug
    def debug_joint_mapping(self):
        env = self.env
        client = env.pb_physics_client

        movable = []

        for joint_id in range(
            p.getNumJoints(env.robot_id, physicsClientId=client)
        ):
            info = p.getJointInfo(
                env.robot_id,
                joint_id,
                physicsClientId=client,
            )

            if info[3] >= 0:  # qIndex: fixed joints have no position DOF
                movable.append((
                    info[3],
                    joint_id,
                    info[1].decode("utf-8"),
                    info[2],
                ))

        movable.sort()
        action_indices = {
            int(joint_id): index
            for index, joint_id in enumerate(env.joint_indices)
        }

        print("IK index | joint ID | joint name | action index")

        for ik_index, (_, joint_id, name, joint_type) in enumerate(movable):
            if joint_type not in {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}:
                raise RuntimeError(
                    f"Joint {name} has multiple DOFs; "
                    "scalar IK mapping is insufficient."
                )

            print(
                ik_index,
                joint_id,
                name,
                action_indices.get(joint_id, "not controlled"),
            )

    def debug_env(self, env):
        ## These are the provided @properties.
        print("Action size = ", env.action_size)
        print("Obs size = ", env.obs_size)
        print("Agent position = ", env.agent_qpos)
        ## below are additional info
        obs = env._get_obs()
        print("Object position = ", obs["object_pos"])
        print("Object xyzw = ", obs["object_xyzw"])
        print("Goal position = ", obs["goal_pos"])
        print("Object point cloud = ", obs["object_pc"])
        print("Object normals = ", obs["object_normals"])
        print("Wall position = ", obs["cube_positions"])