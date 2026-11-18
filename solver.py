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
        self.hold_before_transport = 60   # ticks holding the lifted object before moving
        self.retreat_distance = 1.0       # metres backed away so the held object clears the table before tucking
        self.carry_speed = 0.004          # max base displacement per tick while holding the object
        self.reach_backoff_cells = 7      # cells to back off along the path before reaching to the goal

        # Travel targets retained; navigation rebuilds the map after preparation.
        # Pose used while driving to the table: arm forward, ready to plan a
        # grasp from (the grasp planner's transit search starts here).
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
        # Pose used while carrying the object through the maze, found by a
        # search over joint space: every link stays inside the base footprint
        # (<= 0.29 m forward/sideways) and the gripper is held up at ~1.5 m,
        # >= 0.2 m clear of the body. The drive-to-table pose above reaches
        # 0.84 m forward, which seals the maze bottleneck once the fixed-yaw
        # footprint is inflated; the standard Fetch tuck fits but swings the
        # gripper down beside the base and knocks the object out.
        self.carry_joint_targets = {
            "torso_lift_joint": 0.23,
            "shoulder_pan_joint": -0.59,
            "shoulder_lift_joint": -1.21,
            "upperarm_roll_joint": -2.59,
            "elbow_flex_joint": 1.50,
            "forearm_roll_joint": -1.32,
            "wrist_flex_joint": 1.15,
            "wrist_roll_joint": -0.10,
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
        self.navigate_grasp.set_overhead_obstacles(
            [(lo[0], lo[1], hi[0], hi[1]) for lo, hi in self.overhead_bounds],
            self.overhead_link_reach(self.overhead_bounds),
        )

        ## used for planning
        self.path = None
        self.state = 999
        self.wait = 200

    ## This function is called every time step.
    def get_action(self,env):
        obs = env._get_obs()

        collisions = self.detect_collisions()

        for contact in collisions:
            print("CONTACT/COLLISION:", contact)
        
        if self.state == 999:    ## wait for object to spawn
            # The object spawns at z=5 and needs ~200 steps to land on the table;
            # planning against the falling cloud produced unreachable-airborne goals.
            self.wait -= 1
            if self.wait == 0:
                self.state = 0
            return np.zeros(env.action_size)

        if self.state == 0:
            try:
                decision = self.navigate_grasp.update(
                    obs,
                    self.inflated_occupancy,
                    self.convert_to_grid,
                    self.convert_to_world,
                    self.cell_size,
                )
            except RuntimeError as exc:
                # No usable grasp (e.g. no antipodal pair, or the object fell off
                # the table). Fail into state 4 holding the current pose instead of
                # crashing the whole episode.
                print("Grasp selection failed:", exc)
                self.state = 4
                self.action = np.asarray(obs["qpos"], dtype=float).copy()
                return self.action.copy()

            decision_state = decision["state"]
            print(
                "solver state:", self.state,
                "decision:", decision_state,
            )

            if decision_state == "PREPARE_GRASP":
                # Execute preparation and keep updating the navigation helper.
                self.path = None
                self._path_goal = None
                self.action = np.asarray(
                    decision["action"], dtype=float
                ).copy()
                return self.action.copy()

            if decision_state == "GRASP":
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

                # Latch ONE hold target for the settling phase. Re-commanding the
                # measured qpos every tick lets gravity win: the arm sagged ~28 cm
                # until the gripper rested on the table, and only then counted as
                # "settled", so planning always began from a colliding start state.
                self._grasp_hold_target = np.asarray(obs["qpos"], dtype=float).copy()
                self.action = self._grasp_hold_target.copy()
                return self.action.copy()

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

            # Hold the pose latched on entry to this state (see GRASP transition).
            self.action = self._grasp_hold_target.copy()
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
            # Hold the lifted grasp briefly (confirms the object is secure), then
            # back away from the table before tucking the arm for travel.
            if hasattr(self, "grasp_follower"):
                self.action = self.grasp_follower.hold.copy()
                self._finger_hold = self.grasp_follower.hold[13:15].copy()
            self._hold_ticks = getattr(self, "_hold_ticks", 0) + 1
            if self._hold_ticks >= self.hold_before_transport:
                qpos = np.asarray(obs["qpos"], dtype=float)
                base_xy = np.array([qpos[0] - 2.0, qpos[1]])
                away = base_xy - np.asarray(obs["object_pos"])[:2]
                away /= max(np.linalg.norm(away), 1e-6)
                # Retreat target in qpos coordinates (base x = qpos[0] - 2).
                self._retreat_goal = qpos[:2] + self.retreat_distance * away
                self.state = 5
            return self.action.copy()

        if self.state == 5:
            # Straight-line retreat with the arm and fingers frozen at the hold
            # pose, so the tuck motion cannot sweep the object into the table.
            qpos = np.asarray(obs["qpos"], dtype=float)
            error = self._retreat_goal - qpos[:2]
            step = error * min(1.0, self.carry_speed / max(np.linalg.norm(error), 1e-9))
            self.action[:2] = qpos[:2] + step
            if np.linalg.norm(error) < 0.03:
                self.action[:2] = qpos[:2]
                self.state = 67
                self.path = None
                self._path_goal = None
                # Re-tuck the arm (now carrying the object) and rebuild the
                # navigation map for that pose; the flags are left over from
                # the drive to the table.
                self._travel_pose_complete = False
                self.travel_joint_targets.update(self.carry_joint_targets)
                self._tuck_rate = 0.003   # slow tuck so the held object is not flung
                # The parking map leaves the tabletop out so the base can tuck
                # under it; for transport the whole table must be an obstacle.
                self.create_map(env, False)
                if hasattr(self, "_moving_pose_target"):
                    del self._moving_pose_target
                print("Retreat complete; navigating to goal with object.")
            return self.action.copy()
        # During grasp debugging, retain the failure hold target.
        if self.state == 4:
            print("solver state:", self.state)
            # env.step() needs a (15,) target array; None crashed the episode.
            return self.action.copy()
        # if self.state == 4:
        #     print("solver state:", self.state)
        #     print("Transition to exit")
        #     base_xy = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])
        #     # Choose the retreat goal once, so it does not keep moving.
        #     if not hasattr(self, "_exit_table_goal"):
        #         away = base_xy - np.asarray(obs["object_pos"])[:2]
        #         distance = np.linalg.norm(away)
        #         if distance < 1e-6:
        #             raise PlanningError("Cannot determine retreat direction.")
        #         self._exit_table_goal = base_xy + 0.5 * away / distance

        #         self.path = None
        #         self._path_goal = None

        #     reached_goal = (
        #         np.linalg.norm(base_xy - self._exit_table_goal) < 0.05
        #     )

        #     if reached_goal:
        #         self.create_map(env, False)     # redo map so it treats table as an obstacle
        #         self.state = 67
        #         self.path = None
        #         self._path_goal = None
        #         del self._exit_table_goal
        #         # Return numeric absolute targets, never an exception class.
        #         self.action = np.clip(
        #             np.asarray(self.action, dtype=float),
        #             self.action_lower_bound, self.action_upper_bound,
        #         )
        #         return self.action.copy()
            
        #     goal = self.convert_to_grid(self._exit_table_goal)
        #     return self.navigate(env, goal) 
        
        if self.state == 68:
            return self.reach_goal(obs)

        if self.state == 67:
            print("EXIT 67")
            goal = self.convert_to_grid(obs["goal_pos"][:2])
            action = self.navigate(env, goal)
            if (getattr(self, "_finger_hold", None) is not None
                    and self.curved_nav.status == "reached" and self.path):
                # Arrived at the goal cell with the object: back off along our
                # own path, then place the object on the goal with the arm.
                back = self.path[max(0, len(self.path) - 1 - self.reach_backoff_cells)]
                self._reach_base_goal = self.convert_to_world(back) + np.array([2.0, 0.0])
                self._reach_phase = "backoff"
                self.state = 68
                print("Goal cell reached; backing off to place the object.")
            finger_hold = getattr(self, "_finger_hold", None)
            if finger_hold is not None:
                # Carrying: cap the commanded base displacement per tick. The
                # navigator's waypoint steps (up to ~0.4 m/tick) are fine for an
                # empty gripper but jerk a friction-held object loose.
                qpos = np.asarray(obs["qpos"], dtype=float)
                delta = action[:2] - qpos[:2]
                norm = np.linalg.norm(delta)
                if norm > self.carry_speed:
                    action[:2] = qpos[:2] + delta * (self.carry_speed / norm)
                # Keep squeezing the object; travel poses and navigation steps
                # otherwise command the fingers to their measured (unsqueezed) width.
                action[13:15] = finger_hold
                self.action = action.copy()
            return action

    def reach_goal(self, obs):
        """State 68: put the held object on the goal sphere.

        1. back the base off along the arrival path (straight, slow);
        2. IK (planner's private torso+arm model) for a gripper pose that puts
           the object -- not the gripper -- at the goal, keeping the object's
           offset in the gripper frame fixed;
        3. move the arm there slowly with the fingers still squeezing.
        """
        qpos = np.asarray(obs["qpos"], dtype=float)
        action = self.action.copy()
        action[13:15] = self._finger_hold
        if self._reach_phase == "backoff":
            error = self._reach_base_goal - qpos[:2]
            step = error * min(1.0, self.carry_speed / max(np.linalg.norm(error), 1e-9))
            action[:2] = qpos[:2] + step
            if np.linalg.norm(error) < 0.03:
                action[:2] = qpos[:2]
                self._reach_target = self._solve_reach(obs)
                self._reach_phase = "arm" if self._reach_target is not None else "hold"
                print("Reach IK:", "solved" if self._reach_target is not None else "failed")
        elif self._reach_phase == "arm":
            idx = self.grasp_planner.motion_indices
            error = self._reach_target - qpos[idx]
            action[idx] = qpos[idx] + np.clip(error, -0.003, 0.003)
            if np.max(np.abs(error)) < 0.01:
                self._reach_phase = "hold"
                print("Object placed at goal.")
        self.action = np.clip(action, self.action_lower_bound, self.action_upper_bound)
        return self.action.copy()

    def _solve_reach(self, obs):
        import time as _time
        from scipy.spatial.transform import Rotation
        from custom_libraries.grasp import _ManipulatorIK
        planner = self.grasp_planner
        qpos = np.asarray(obs["qpos"], dtype=float)
        robot, client = planner.robot, planner.client
        ee = next(j for j in range(p.getNumJoints(robot, physicsClientId=client))
                  if p.getJointInfo(robot, j, physicsClientId=client)[12].decode() == "gripper_link")
        state = p.getLinkState(robot, ee, computeForwardKinematics=True, physicsClientId=client)
        g_pos, g_rot = np.asarray(state[4]), Rotation.from_quat(state[5])
        offset_local = g_rot.inv().apply(np.asarray(obs["object_pos"]) - g_pos)
        goal = np.asarray(obs["goal_pos"], dtype=float)
        idx = planner.motion_indices
        lower, upper = planner.lower[idx].copy(), planner.upper[idx].copy()
        seed = qpos[idx].copy()
        lower[planner.wrap] = np.maximum(lower[planner.wrap], seed[planner.wrap] - np.pi)
        upper[planner.wrap] = np.minimum(upper[planner.wrap], seed[planner.wrap] + np.pi)
        ik = _ManipulatorIK(planner, qpos)
        try:
            # Current gripper orientation first, then level/forward variants.
            candidates = [g_rot] + [Rotation.from_euler("xyz", [r, 0, y])
                                    for y in (0.0, 0.5, -0.5) for r in (0.0, np.pi/2, -np.pi/2)]
            for rot in candidates:
                target = goal - rot.apply(offset_local)
                q = ik.solve(target, rot.as_quat(), seed, lower, upper,
                             _time.monotonic() + 2.0, restarts=12)
                if q is not None:
                    return q
        finally:
            ik.close()
        return None

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
        rate = getattr(self, "_tuck_rate", 0.05)
        action[arm] = qpos[arm] + np.clip(error, -rate, rate)
        if hasattr(self, "_tuck_rate"):
            # Carrying: lower the torso gradually too, instead of jumping to
            # the travel height with the object still in hand.
            t = self._travel_torso_index
            action[t] = qpos[t] + np.clip(target[t] - qpos[t], -0.002, 0.002)
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
        # Boxes skipped by the 2-D base map because the base fits underneath
        # (e.g. the table top). The torso does NOT fit underneath, so these are
        # kept for the parking clearance check in NavigateToGrasp.
        self.overhead_bounds = []

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
                        self.overhead_bounds.append((np.asarray(lower), np.asarray(upper)))
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

    def overhead_link_reach(self, overhead_bounds):
        """Horizontal forward reach (m) from the base centre of every robot link
        whose height overlaps an overhead obstacle (in practice: the torso at
        table-top height). Measured once from the actual collision geometry."""
        if not overhead_bounds:
            return 0.0
        client, robot = self.env.pb_physics_client, self.env.robot_id
        z_lo = min(lo[2] for lo, _ in overhead_bounds)
        z_hi = max(hi[2] for _, hi in overhead_bounds)
        base_xy = np.array([self.env.agent_qpos[0] - 2.0, self.env.agent_qpos[1]])
        yaw = float(self.env.agent_qpos[2])
        forward = np.array([np.cos(yaw), np.sin(yaw)])
        reach = 0.0
        for link in range(-1, p.getNumJoints(robot, physicsClientId=client)):
            if not p.getCollisionShapeData(robot, link, physicsClientId=client):
                continue
            lo, hi = p.getAABB(robot, link, physicsClientId=client)
            if hi[2] < z_lo or lo[2] > z_hi:
                continue
            corners = np.array([[x, y] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])])
            reach = max(reach, float(np.max((corners - base_xy) @ forward)))
        return reach

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

    def detect_collisions(self, ignore_bodies=()):
        contacts = p.getContactPoints(
            bodyA=self.env.robot_id,
            physicsClientId=self.env.pb_physics_client,
        )

        return [
            {
                "other_body": c[2],
                "robot_link": c[3],
                "other_link": c[4],
                "distance": c[8],       # metres; negative = penetration
                "normal_force": c[9],   # newtons
            }
            for c in contacts
            if c[2] not in ignore_bodies
            and (c[8] <= 0.0 or c[9] > 0.0)
        ]
