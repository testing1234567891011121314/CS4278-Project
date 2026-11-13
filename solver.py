import numpy as np
import pybullet as p
from utils import closest_joint_values  # Handles joint angle wrapping

## import custom libraries
from custom_libraries.curved_navigator import CurvedNavigator 
from custom_libraries.navigate_to_grasp import NavigateToGrasp
from custom_libraries.grasp import GraspPlanner, GraspFollower
from custom_libraries.grasp import PlanningError
from custom_libraries.a_star import a_star
from custom_libraries.wall_projection import build_wall_collision_grid

class Solver:
    def __init__(self, env, *, travel_joint_targets=None):
        self.env = env
        self.robot_radius = self.get_base_radius()
        if not np.isfinite(self.robot_radius) or self.robot_radius <= 0:
            raise ValueError("robot_radius must be a positive number in metres.")
        
        # self.action = env.agent_qpos # np.zeros(self.env.action_size)
        self.action = np.asarray(env.agent_qpos, dtype=float).copy()
        self.action_scale = np.array([0.01] * env.action_size)
        self.action_scale[:3] = 0.02  # root movement
        self.action_scale[-2:] = 0.005  # gripper
        self.action_upper_bound = env.joint_upper
        self.action_lower_bound = env.joint_lower
        self.turn = 0
        self.forward = 0
        self.yaw = 0
        self.torso_lift = 0.0     # max lift 0.38615 metres
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

        # Travel targets retained; navigation rebuilds the map after preparation.
        self.travel_joint_targets = {
            "torso_lift_joint": 0.1,
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.2,
            "upperarm_roll_joint": 0.0,
            "elbow_flex_joint": 1.0,
            "forearm_roll_joint": 0.0,
            "wrist_flex_joint": 1.0,
            "wrist_roll_joint": 0.0,
        }

        if travel_joint_targets is not None:
            unknown = set(travel_joint_targets) - set(self.travel_joint_targets)
            if unknown:
                raise ValueError(f"Unsupported travel joints: {sorted(unknown)}")
            self.travel_joint_targets.update(travel_joint_targets)
        names = {
            p.getJointInfo(env.robot_id, j, physicsClientId=env.pb_physics_client)[1].decode(): i
            for i, j in enumerate(env.joint_indices)
        }
        self._travel_indices = np.array(
            [names[name] for name in self.travel_joint_targets], dtype=int
        )
        self._travel_torso_index = names["torso_lift_joint"]
        # Validate the configured travel targets.
        self._travel_target(self.action)

        self.create_map(env, True)
        self.inflated_occupancy = self.create_navigable_map()
        
        client_id = env.pb_physics_client

        self.grasp_planner = GraspPlanner(
            env, robot_urdf_path="assets/fetch/fetch.urdf",
            physics_client_id=client_id,
        )

        self.curved_nav = CurvedNavigator(self)
        self.navigate_grasp = NavigateToGrasp(env, self.grasp_planner)

        ## used for planning
        self.path = None
        self.state = 999
        self.wait = 200

    ## This function is called every time step.
    def get_action(self,env):
        obs = env._get_obs()
        
        if self.state == 999:    ## wait for object to spawn
            # The object spawns at z=5 and needs ~200 steps to land on the table;
            # planning against the falling cloud produced unreachable-airborne goals.
            self.wait -= 1
            if self.wait == 0:
                self.state = 0
            return np.zeros(env.action_size)

        if self.state == 0:
            decision = self.navigate_grasp.update(
                obs,
                self.inflated_occupancy,
                self.convert_to_grid,
                self.convert_to_world,
                self.cell_size,
            )

            decision_state = decision["state"]
            print(
                "solver state:", self.state,
                "decision:", decision_state,
            )

            # if decision_state == "PREPARE_GRASP":
            #     # Remain in solver state 0 so update() runs next timestep.
            #     # The helper turns the base and waits for it to settle.
            #     self.path = None
            #     self._path_goal = None
            #     # self.waypoint_index = 0
            #     # self.curved_nav.reset()

            #     return np.asarray(
            #         decision["action"], dtype=float
            #     ).copy()

            if decision_state in {"PREPARE_GRASP", "GRASP"}:
                # Only the helper decides when preparation is complete.
                self.state = 1
                self._grasp_settle_ticks = 0
                self._grasp_last_object_pos = None

                # Record the actual parked WORLD position for failed retries.
                # navigation_goal is None when decision_state == "GRASP".
                base_position, _ = self.navigate_grasp.get_base_pose(obs)
                self.grasp_attempt_goal = tuple(base_position[:2])

                self.path = None
                self._path_goal = None
                self.waypoint_index = 0
                self.curved_nav.reset()

                return np.asarray(obs["qpos"], dtype=float).copy()

            if decision_state in {
                "NAVIGATE_TO_OBJECT",
                "NAVIGATE_TO_GRASP",
            }:
                goal_world = decision["navigation_goal"]
                if goal_world is None:
                    raise RuntimeError(
                        f"{decision_state} requires a navigation goal."
                    )

                goal = self.convert_to_grid(goal_world)
                return self.navigate(env, goal)

            raise RuntimeError(
                f"Unknown navigation/grasp decision: {decision_state}"
            )
        
        if self.state == 1:
            print("solver state:", self.state)
            from pprint import pprint
            import time

            # Hold the measured pose while waiting for settling.
            self.action = np.asarray(obs["qpos"], dtype=float).copy()
            object_pos = np.asarray(obs["object_pos"], dtype=float)

            previous = getattr(self, "_grasp_last_object_pos", None)
            self._grasp_last_object_pos = object_pos.copy()

            settled = (
                previous is not None
                and np.linalg.norm(object_pos - previous) < 0.001
                and np.max(np.abs(obs["qvel"])) < 0.05
            )

            self._grasp_settle_ticks = (
                getattr(self, "_grasp_settle_ticks", 0) + 1
                if settled else 0
            )

            if self._grasp_settle_ticks < 12:
                return self.action.copy()

            # Planning starts only after settling.
            started = time.monotonic()
            print("[PLAN] Starting grasp planning", flush=True)

            plan = self.grasp_planner.plan(obs)

            print(
                f"[PLAN] Finished in {time.monotonic() - started:.2f}s; "
                f"success={plan is not None}",
                flush=True,
            )
            pprint(self.grasp_planner.last_stats)

            if plan is None:
                self.grasp_error = "No plan found within the grasp search budget."
                self.state = 4
                return self.action.copy()

            self.grasp_plan = plan
            self.grasp_follower = GraspFollower(self.grasp_planner, plan)
            self.state = 2
            return self.action.copy()

        if self.state == 2:
            print("solver state:", self.state)
            self.action = self.grasp_follower.step(obs)
            if self.grasp_follower.status == "done":
                self.state = 3
                print("Grasp and lift complete; holding object.")
            elif self.grasp_follower.status == "failed":
                self.grasp_error = self.grasp_follower.error
                self.state = 4
                print("Grasp stopped:", self.grasp_error)
            return self.action.copy()

        if self.state == 3:
            print("solver state:", self.state)
            # Terminal hold. Payload-aware transport is a separate planning
            # task; the previous transition reopened the grasp and reset the
            # arm to an unchecked travel pose.
            if hasattr(self, "grasp_follower"):
                self.action = self.grasp_follower.hold.copy()
            return self.action.copy()
        
        if self.state == 4:
            print("solver state:", self.state)
            print("Transition to exit")
            base_xy = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])
            # Choose the retreat goal once, so it does not keep moving.
            if not hasattr(self, "_exit_table_goal"):
                away = base_xy - np.asarray(obs["object_pos"])[:2]
                distance = np.linalg.norm(away)
                if distance < 1e-6:
                    raise PlanningError("Cannot determine retreat direction.")
                self._exit_table_goal = base_xy + 0.5 * away / distance

                self.path = None
                self._path_goal = None

            reached_goal = (
                np.linalg.norm(base_xy - self._exit_table_goal) < 0.05
            )

            if reached_goal:
                self.create_map(env, False)     # redo map so it treats table as an obstacle
                self.state = 67
                self.path = None
                self._path_goal = None
                del self._exit_table_goal
                # Return numeric absolute targets, never an exception class.
                self.action = np.clip(
                    np.asarray(self.action, dtype=float),
                    self.action_lower_bound, self.action_upper_bound,
                )
                return self.action.copy()
            
            goal = self.convert_to_grid(self._exit_table_goal)
            return self.navigate(env, goal) 
        
        if self.state == 67:
            print("EXIT 67")
            goal = self.convert_to_grid(obs["goal_pos"][:2])
            return self.navigate(env, goal)

    def navigate(self, env, goal):
        obs = env._get_obs()
        qpos = obs["qpos"]

        # Original navigation sequence: prepare, rebuild, then navigate.
        if not getattr(self, "_travel_pose_complete", False):
            pose_action = self.moving_pose(obs)
            if self.moving_pose_ready:
                self.inflated_occupancy = self.create_navigable_map()
                self._travel_pose_complete = True
                self.path = None
                self._path_goal = None
                self.waypoint_index = 0
                self.curved_nav.reset()
                print("Travel pose ready; rebuilt navigation map.")
            return pose_action

        base_xy = np.array([qpos[0] - 2.0, qpos[1]])

        start = self.convert_to_grid(base_xy)

        goal_key = tuple(np.asarray(goal, dtype=int))
        if goal_key != getattr(self, "_path_goal", None):
            self.path = None
            self._path_goal = goal_key
            self.waypoint_index = 0
            self.curved_nav.reset()

        if self.path is None:
            path = a_star(start, goal_key, self.inflated_occupancy)
            print("Path length:", 0 if path is None else len(path))
            if path is None or len(path) == 0:
                grid = self.inflated_occupancy
                def cell_status(cell):
                    r, c = cell
                    if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
                        return "outside map"
                    return "blocked" if grid[r, c] else "free"
                raise PlanningError(
                    f"No route: start={tuple(start)} ({cell_status(start)}), "
                    f"goal={goal_key} ({cell_status(goal_key)}). "
                    "Inspect map connectivity and the projected travel footprint. "
                    "Restoring the projection state does not make a wide pose narrower."
                )
            self.path = path
            self.waypoint_index = 0
            self.curved_nav.reset()
            # Continue below: env.step requires a numeric action, not an exception class.


        self.forward = 0
        self.turn = 0
        self.yaw = 0

        # Anchor translation targets to the measured position.
        self.action[:2] = qpos[:2]

        if self.path:
            step_xy = self.curved_nav.step(base_xy, self.path)

            self.forward = step_xy[0] / self.action_scale[0]
            self.turn = step_xy[1] / self.action_scale[1]
            self.waypoint_index = self.curved_nav.grid_index

            # Print only when the navigation status changes.
            status = self.curved_nav.status
            if status != getattr(self, "_last_nav_status", None):
                print("Navigation:", status)
                self._last_nav_status = status

        unit_action = [
            self.forward, self.turn, self.yaw,
            self.torso_lift, self.head_pan, self.head_tilt,
            self.shoulder_pan, self.shoulder_lift,
            self.upperarm_roll, self.elbow_flex,
            self.forearm_roll, self.wrist_flex, self.wrist_roll,
            self.gripper_open, self.gripper_open,
        ]

        self.action = np.clip(
            self.action + self.action_scale * np.asarray(unit_action),
            self.action_lower_bound,
            self.action_upper_bound,
        )
        return self.action.copy()

    def _travel_target(self, qpos):
        """Return the configured travel target, preserving all other joints."""
        target = np.asarray(qpos, dtype=float).copy()
        for i, value in zip(self._travel_indices, self.travel_joint_targets.values()):
            if (not np.isfinite(value) or value < self.action_lower_bound[i]
                    or value > self.action_upper_bound[i]):
                raise ValueError(f"Travel target outside joint limits: index={i}, value={value}")
            target[i] = value
        return target

    def moving_pose(self, obs):
        """Original arm interpolation/readiness, using the configured tuck pose."""
        qpos = np.asarray(obs["qpos"], dtype=float)
        if not hasattr(self, "_moving_pose_target"):
            self._moving_pose_target = self._travel_target(qpos)
        target = self._moving_pose_target
        # The previous helper waited for the seven arm joints only.
        arm = self._travel_indices[
            self._travel_indices != self._travel_torso_index
        ]
        action = target.copy()
        error = target[arm] - qpos[arm]
        action[arm] = qpos[arm] + np.clip(error, -0.05, 0.05)
        qvel = np.asarray(obs["qvel"], dtype=float)
        self.moving_pose_ready = bool(
            np.max(np.abs(error)) < 0.03
            and np.max(np.abs(qvel[arm])) < 0.05
        )
        self.action = np.clip(
            action, self.action_lower_bound, self.action_upper_bound
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
    def create_map(self, env, is_2d=True):
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
        extra_bounds = self.get_extra_obstacle_bounds(cube_positions, is_2d)
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

    ### Helper functions for querying pybullet directly to find the table. ###
    def get_extra_obstacle_bounds(self, cube_positions, is_2d):
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
                # The 2-D map plans the BASE only. A box entirely above base
                # height (table top underside ~0.59 m vs base ~0.2 m tall)
                # does not block it -- inflating those footprints sealed the
                # reach annulus around the table and stranded the robot 1.0 m
                # out. Table legs start at z=0 and still count.
                if is_2d and lower[2] >= 0.35:  # allow parking under table if navigating to object, disallow when navigating to exit
                        continue
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