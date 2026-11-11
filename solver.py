import numpy as np
import pybullet as p
from utils import closest_joint_values  # Handles joint angle wrapping

## import custom libraries
from custom_libraries.curved_navigator import CurvedNavigator 
from custom_libraries.navigate_to_grasp import NavigateToGrasp
from custom_libraries.antipodal import AntipodalGraspPlanner
from custom_libraries.a_star import a_star
from custom_libraries.wall_projection import build_wall_collision_grid

class Solver:
    def __init__(self, env):
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
        
        client_id = env.pb_physics_client

        gripper_link_index = next(
            j
            for j in range(
                p.getNumJoints(env.robot_id, physicsClientId=client_id)
            )
            if p.getJointInfo(
                env.robot_id, j, physicsClientId=client_id
            )[12].decode() == "gripper_link"
        )

        self.grasp_planner = AntipodalGraspPlanner(
            robot_id=env.robot_id,
            ee_link_index=gripper_link_index,
            physics_client_id=client_id,
            action_joint_ids=env.joint_indices,
        )

        self.curved_nav = CurvedNavigator(self)
        self.navigate_grasp = NavigateToGrasp(env, self.grasp_planner)

        ## used for planning
        self.path = None
        self.state = 0

    ## This function is called every time step.
    def get_action(self,env):
        obs = env._get_obs()

        if self.state == 0:
            decision = self.navigate_grasp.update(obs, self.inflated_occupancy, self.convert_to_grid, self.convert_to_world, self.cell_size)
            goal_world = decision["navigation_goal"]
            base_xy = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])

            print("solver state:", self.state, "decision:", decision["state"])
            arrived = (
                goal_world is not None
                and np.linalg.norm(base_xy - goal_world) < 0.10
            )
            if decision["state"] == "GRASP" or arrived:
                self.state = 1
                self.grasp_attempt_goal = (
                    tuple(np.round(np.asarray(goal_world), 3))
                    if goal_world is not None else None
                )
                self.path = None
                self._path_goal = None
                self.waypoint_index = 0
                self.curved_nav.reset()
                return np.asarray(obs["qpos"]).copy()

            goal = self.convert_to_grid(decision["navigation_goal"])
            return self.navigate(env, goal)
        
        if self.state == 1: # plan grasp. dont move until planning is done
            print("solver state:", self.state)
            if self.grasp_planner.start(obs, object_id=env.object_id):
                self.state = 2
            return self.action.copy()
        
        if self.state == 2: # actual grasping
            print("solver state:", self.state)
            return None
        
        if self.state == 3:    
            print("solver state:", self.state)
            goal = self.convert_to_grid(obs["goal_pos"][:2])
            return self.navigate(env, goal)
        
    def execute_grasp(self, obs):
        qpos = np.asarray(obs["qpos"], dtype=float)

        # Plan once, like `if self.path is None` in navigate().
        if getattr(self, "grasp_path", None) is None:
            client = self.env.pb_physics_client
            robot_id = self.env.robot_id

            joint_info = {
                j: p.getJointInfo(
                    robot_id, j, physicsClientId=client
                )
                for j in range(p.getNumJoints(
                    robot_id, physicsClientId=client
                ))
            }

            ee_matches = [
                j for j, info in joint_info.items()
                if info[12].decode("utf-8") == "gripper_link"
            ]
            if len(ee_matches) != 1:
                raise RuntimeError("Expected exactly one gripper_link.")

            # Map PyBullet joint IDs to your action-vector indices.
            arm_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "upperarm_roll_joint",
                "elbow_flex_joint",
                "forearm_roll_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
            ]
            joint_ids = {
                info[1].decode("utf-8"): j
                for j, info in joint_info.items()
            }
            action_index = {
                joint_ids[name]: i
                for i, name in enumerate(arm_names, start=6)
            }

            candidates = self.grasp_planner.sample_antipodal_grasps(
                points=obs["object_pc"],
                normals=obs["object_normals"],
                num_samples=500,
            )

            object_pos = np.asarray(obs["object_pos"], dtype=float)
            self.grasp_proved_unreachable = False
            if object_pos[2] > 1.0:
                # Object still falling from its z=5 spawn: an IK attempt now
                # would prove a perfectly good cell un-graspable and blacklist
                # it for the rest of the run.
                print("object airborne at z=%.2f; deferring grasp" % object_pos[2])
                return None

            grasp = self.grasp_planner.select_optimal_grasp(
                robot_id=robot_id,
                ee_link_index=ee_matches[0],
                candidates=candidates,
                strategy="greedy_first",
                max_candidates=25,
                support_z=float(np.min(np.asarray(obs["object_pc"])[:, 2])),
            )

            if grasp is None:
                # Parked pose proven un-graspable; the caller blacklists
                # this cell and navigates to another approach pose.
                # Nothing was even tried (empty candidate list): this says
                # nothing about the cell, so don't poison it.
                stats = self.grasp_planner.ik_stats.get("greedy_first", {})
                self.grasp_last_stats = stats
                self.grasp_proved_unreachable = stats.get("calls", 0) > 0
                print("grasp IK failed from parked pose:", stats)
                print("  candidates:", len(candidates),
                      "object_pos:", np.round(object_pos, 3),
                      "base_xy:", np.round(np.asarray(qpos[:3]), 3))
                for f in stats.get("failures", [])[:3]:
                    print("  failure point:", f)
                return None

            arm_indices = np.array([
                action_index[j] for j in grasp["arm_joint_ids"]
            ], dtype=int)
            arm_target = np.asarray(
                grasp["joint_angles"], dtype=float
            )

            lower = np.asarray(self.action_lower_bound)
            upper = np.asarray(self.action_upper_bound)

            if (
                arm_target.shape != arm_indices.shape
                or not np.all(np.isfinite(arm_target))
                or np.any(arm_target < lower[arm_indices])
                or np.any(arm_target > upper[arm_indices])
            ):
                raise RuntimeError("Invalid grasp joint targets.")

            # Each row is a complete absolute joint-position target.
            # Preserve the current base, torso and head positions.
            open_pose = np.clip(qpos.copy(), lower, upper)
            open_pose[-2:] = upper[-2:]

            reach_pose = open_pose.copy()
            reach_pose[arm_indices] = arm_target

            close_pose = reach_pose.copy()
            close_pose[-2:] = lower[-2:]

            lift_pose = close_pose.copy()
            lift_pose[3] = upper[3]

            self.grasp_path = np.stack([
                open_pose,
                reach_pose,
                close_pose,
                lift_pose,
            ])
            self.grasp_waypoint_index = 0
            self._grasp_waypoint_ticks = 0
            self.grasp_motion_complete = False
            self._grasp_evidence_printed = False

        target = self.grasp_path[self.grasp_waypoint_index]

        # Continue holding the final pose after closing.
        if self.grasp_motion_complete:
            if not getattr(self, "_grasp_evidence_printed", False):
                self._grasp_evidence_printed = True
                pts = p.getContactPoints(
                    bodyA=self.env.robot_id,
                    bodyB=self.env.object_id,
                    physicsClientId=self.env.pb_physics_client)
                obj_z = float(np.asarray(obs["object_pos"])[2])
                print(f"GRASP CLOSED: robot-object contacts={len(pts)} "
                      f"fingers={np.round(qpos[-2:], 4)} "
                      f"object z={obj_z:.3f} (table top ~0.626)")
            self.action = target.copy()
            return self.action.copy()

        # Convert the current waypoint into one bounded action.
        action = qpos + np.clip(
            target - qpos,
            -self.action_scale,
            self.action_scale,
        )

        # Keep commanding the planned base, torso and head pose.
        action[:6] = target[:6]

        self._grasp_waypoint_ticks += 1

        if self.grasp_waypoint_index == 0:
            # OPEN: check the finger positions.
            reached = np.max(
                np.abs(target[-2:] - qpos[-2:])
            ) < 0.002

        elif self.grasp_waypoint_index == 1:
            # REACH: check arm position and settling.
            reached = (
                np.max(np.abs(target[6:13] - qpos[6:13])) < 0.02
                and np.max(np.abs(
                    np.asarray(obs["qvel"])[6:13]
                )) < 0.05
            )

        elif self.grasp_waypoint_index == 2:
            # CLOSE: ramp the prismatic fingers shut slowly -- the ±0.05
            # action scale slams the 0.024-range joints in one step, which
            # bats the light slab sideways before the pads arrive.
            action[6:13] = target[6:13]
            action[-2:] = qpos[-2:] + np.clip(
                target[-2:] - qpos[-2:], -0.0004, 0.0004
            )
            reached = self._grasp_waypoint_ticks >= 160

        else:
            # LIFT: torso rises with the fingers still closed. Ramp it --
            # a torso jump from the base-override below excites arm
            # oscillation that shakes the object out of the pads.
            action[3] = float(
                qpos[3] + np.clip(target[3] - qpos[3], -0.008, 0.008)
            )
            action[6:13] = target[6:13]
            reached = (
                abs(target[3] - qpos[3]) < 0.02
                and abs(float(np.asarray(obs["qvel"])[3])) < 0.05
            )

        if reached:
            if self.grasp_waypoint_index < len(self.grasp_path) - 1:
                self.grasp_waypoint_index += 1
                self._grasp_waypoint_ticks = 0
            else:
                self.grasp_motion_complete = True

        elif self._grasp_waypoint_ticks > 600:
            raise RuntimeError(
                f"Grasp stalled at waypoint {self.grasp_waypoint_index}."
            )

        self.action = np.clip(
            action,
            self.action_lower_bound,
            self.action_upper_bound,
        )
        return self.action.copy()

    def navigate(self, env, goal):
        obs = env._get_obs()
        qpos = obs["qpos"]

        # Keep moving toward the test pose until it has settled.
        if not getattr(self, "_travel_pose_complete", False):
            pose_action = self.moving_pose(obs)

            if self.moving_pose_ready:
                self._travel_pose_complete = True

                # Rebuild using the robot's measured, settled posture.
                self.inflated_occupancy = self.create_navigable_map()
                self.path = None
                self.waypoint_index = 0
                self.curved_nav.reset()

            # Hold the base still throughout pose preparation.
            return pose_action
        
        base_xy = np.array([qpos[0] - 2.0, qpos[1]])

        start = self.convert_to_grid(base_xy)

        # Plan once using the separate A* module.
        if self.path is None:
            self.path = a_star(start, goal, self.inflated_occupancy)
            self.waypoint_index = 0
            self.curved_nav.reset()
            print("Path length:", len(self.path))

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
                # The 2-D map plans the BASE only. A box entirely above base
                # height (table top underside ~0.59 m vs base ~0.2 m tall)
                # does not block it -- inflating those footprints sealed the
                # reach annulus around the table and stranded the robot 1.0 m
                # out. Table legs start at z=0 and still count.
                if lower[2] >= 0.35:
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