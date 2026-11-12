import numpy as np
import pybullet as p
from utils import closest_joint_values  # Handles joint angle wrapping

## import custom libraries
from custom_libraries.curved_navigator import CurvedNavigator 
from custom_libraries.navigate_to_grasp import NavigateToGrasp
from custom_libraries.grasp import GraspPlanner, GraspFollower
from custom_libraries.grasp import PlanningError
from custom_libraries.a_star import a_star
from custom_libraries.wall_projection import build_wall_collision_grid, robot_xy_polygons

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

        ## final approach (see fine_approach): creep from the coarse grid goal
        ## to a standoff of grasp_standoff metres from the grasp point.
        self.grasp_standoff = 0.60      # base origin -> grasp centre, metres
        self.table_margin = 0.03        # continuous clearance kept to obstacles
        self._fine_active = False

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
            if self._fine_active:
                action, done = self.fine_approach(obs)
                if not done:
                    return action
                self._fine_active = False
                self.state = 1
                self._grasp_settle_ticks = 0
                self._grasp_last_object_pos = None
                self.grasp_attempt_goal = self._fine_goal_key
                self.path = None
                self._path_goal = None
                self.waypoint_index = 0
                self.curved_nav.reset()
                return np.asarray(obs["qpos"]).copy()

            decision = self.navigate_grasp.update(obs, self.inflated_occupancy, self.convert_to_grid, self.convert_to_world, self.cell_size)
            goal_world = decision["navigation_goal"]
            base_xy = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])

            print("solver state:", self.state, "decision:", decision["state"])
            arrived = (
                goal_world is not None
                and np.linalg.norm(base_xy - goal_world) < 0.10
            )
            if (decision["state"] == "GRASP" or arrived) and not self._fine_active:
                # Coarse (0.1 m grid) navigation is done. Creep closer before
                # grasp planning: the grid keeps the base ~0.25 m further from
                # the table than it needs to be, i.e. at the edge of arm reach.
                grasp_xy = decision.get("grasp_position_world")
                if grasp_xy is None:
                    grasp_xy = obs["object_pos"]
                self._fine_goal_key = (
                    tuple(np.round(np.asarray(goal_world), 3))
                    if goal_world is not None else None
                )
                self.start_fine_approach(obs, np.asarray(grasp_xy, float)[:2])
                action, done = self.fine_approach(obs)
                if not done:
                    return action
                self._fine_active = False
            if decision["state"] == "GRASP" or arrived:
                self.state = 1
                self._grasp_settle_ticks = 0
                self._grasp_last_object_pos = None
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

            ### THE BELOW BLOCK IS FOR DEBUG ###
            from pathlib import Path
            import json

            pose_path = Path("/home/hejun/GitHub/CS4278-Project/parked_qpos.json")
            measured_pose = {
                name: float(value)
                for name, value in zip(
                    self.grasp_planner.action_names, obs["qpos"]
                )
            }
            pose_path.write_text(
                json.dumps(measured_pose, indent=2),
                encoding="utf-8",
            )
            print(f"[DEBUG] Saved pre-grasp posture to {pose_path}", flush=True)
            ### THE ABOVE BLOCK IS FOR DEBUG ###

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
            # Return numeric absolute targets, never an exception class.
            self.action = np.clip(
                np.asarray(self.action, dtype=float),
                self.action_lower_bound, self.action_upper_bound,
            )
            # return self.action.copy()
            return PlanningError

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

    ### Final approach: exact (continuous) clearance instead of the 0.1 m grid ###
    def _footprint_boxes(self):
        """[(obstacle_lower, obstacle_upper, footprint_min_xy, footprint_max_xy)].

        footprint = XY extent, relative to the base origin, of the robot links
        in that obstacle's height band at the robot's current pose.
        """
        boxes = []
        z_pad = 0.03
        for lower, upper in self.extra_bounds:
            polys = robot_xy_polygons(
                self.env, (lower[2] - z_pad, upper[2] + z_pad)
            )
            if not polys:
                continue
            pts = np.vstack(polys)
            boxes.append((lower, upper, pts.min(axis=0), pts.max(axis=0)))
        return boxes

    def _pose_clear(self, base_xy, boxes, margin):
        base_xy = np.asarray(base_xy, float)
        for lower, upper, fmin, fmax in boxes:
            lo, hi = base_xy + fmin, base_xy + fmax
            if (lo[0] < upper[0] + margin and hi[0] > lower[0] - margin
                    and lo[1] < upper[1] + margin and hi[1] > lower[1] - margin):
                return False
        r, c = self.convert_to_grid(base_xy)
        rows, cols = self._wall_blocked.shape
        if not (0 <= r < rows and 0 <= c < cols) or self._wall_blocked[r, c]:
            return False
        return True

    def start_fine_approach(self, obs, grasp_xy):
        """Pick the closest safe standoff pose on the line base -> grasp.

        Desired pose: base y aligned with the grasp, base x grasp_standoff short
        of it (the arm points along +x). The straight segment from the current
        base is walked in 1 cm steps and cut at the first pose that would
        come within table_margin of an obstacle in the torso/arm height band
        (or enter a wall-blocked cell), so the target is never unsafe.
        """
        qpos = np.asarray(obs["qpos"], float)
        start = np.array([qpos[0] - 2.0, qpos[1]])
        desired = np.array([grasp_xy[0] - self.grasp_standoff, grasp_xy[1]])
        boxes = self._footprint_boxes()
        n = max(2, int(np.ceil(np.linalg.norm(desired - start) / 0.01)))
        target = start.copy()
        if self._pose_clear(start, boxes, self.table_margin):
            for t in np.linspace(0.0, 1.0, n + 1):
                point = start + t * (desired - start)
                if not self._pose_clear(point, boxes, self.table_margin):
                    break
                target = point
        self._fine_target = target
        self._fine_active = True
        print(f"Fine approach: start={np.round(start, 3)} "
              f"desired={np.round(desired, 3)} target={np.round(target, 3)}")

    def fine_approach(self, obs, max_step=0.04):
        """Move the base (absolute joint targets) to _fine_target. -> (action, done)."""
        qpos = np.asarray(obs["qpos"], float)
        base = np.array([qpos[0] - 2.0, qpos[1]])
        error = self._fine_target - base
        distance = np.linalg.norm(error)
        speed = np.linalg.norm(np.asarray(obs["qvel"], float)[:2])
        if distance < 0.01 and speed < 0.02:
            return qpos.copy(), True
        step = error if distance <= max_step else error * (max_step / distance)
        action = qpos.copy()          # hold every other joint where it is
        action[:2] = qpos[:2] + step
        action = np.clip(action, self.action_lower_bound, self.action_upper_bound)
        return action, False

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
            target[9] = -1.0    # elbow_flex
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
        self.extra_bounds = extra_bounds          # kept (with z extents) for create_navigable_map
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
        self._wall_blocked = wall_blocked.astype(bool)

        # Additional obstacles (table, ...): inflate each one by the XY
        # footprint of ONLY the robot links that share its height band. The
        # table has a single 0.05 m top slab (z 0.574-0.626, no legs), so the
        # low base/wheels pass underneath, but torso_lift_link / torso_fixed_link
        # (z ~0.4-0.98, reaching +0.09 m ahead of the base origin) would hit it.
        # Footprints use the robot's current (travel) pose, so call this again
        # after the torso/arm settles (navigate() already does).
        rows, cols = self.extra_occupancy.shape
        extra_blocked = np.zeros((rows, cols), dtype=bool)
        z_pad = 0.03
        for lower, upper in self.extra_bounds:
            single = np.zeros((rows, cols), dtype=np.uint8)
            self.mark_obstacle_bounds(single, [(lower, upper)])
            extra_blocked |= build_wall_collision_grid(
                self.env, single, self.cell_size,
                safety_margin=safety_margin,
                z_range=(lower[2] - z_pad, upper[2] + z_pad),
            ).astype(bool)

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
                # Do NOT discard obstacles that sit above base height. The
                # table is only a top slab (z 0.574-0.626) with no collision
                # legs, so a "lower z >= 0.35" filter removed it entirely and
                # the base/torso were routed straight into it. Height is
                # handled per obstacle in create_navigable_map().
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