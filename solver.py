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
        # Extra finger closing (metres per finger) commanded once the object is
        # lifted. Grip force is linear in how far the finger command sits inside
        # the object: measured 6.6 N per mm, steady, 50 N at the grasp's own
        # squeeze and ~250 N with the fingers commanded shut. Friction, and so
        # every speed the hold tolerates, scales with it.
        self.carry_grip_extra = 0.030
        self.carry_accel = 0.0005         # change of that displacement per tick (speed ramp)
        self.tuck_rate = 0.003            # rad per tick when moving the arm into the carry pose with the object
        self.reach_rate = 0.003           # rad per tick for the final arm move to the goal
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
        # Pose used while carrying the object through the maze, found by an IK
        # search over gripper positions above the base (tools/ are not needed
        # to reproduce it: any pose meeting these conditions will do):
        # - every link stays inside the base footprint (<= 0.289 m), so the
        #   fixed-yaw wall inflation does not seal the maze bottleneck (the
        #   drive-to-table pose reaches 0.84 m forward);
        # - the gripper points straight up at 1.6 m with the palm under the
        #   object. The object creeps in the fingers while the base drives; in
        #   the previous carry pose the closing axis pointed down and the
        #   object slid sideways off the 24 mm-tall pads after 10-14 mm. Here
        #   gravity takes any creep into the palm instead.
        self.carry_joint_targets = {
            "torso_lift_joint": 0.04,
            "shoulder_pan_joint": 0.924,
            "shoulder_lift_joint": -1.20,
            "upperarm_roll_joint": -0.55,
            "elbow_flex_joint": -1.418,
            "forearm_roll_joint": -0.2,
            "wrist_flex_joint": 1.125,
            "wrist_roll_joint": 1.257,
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
        self._wrist_roll_index = names["wrist_roll_joint"]
        self.align_grip_to_travel = True   # turn the fingers to face the direction of travel while carrying
        self.grip_align_rate = 0.1         # rad per tick
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

    def _check_holding(self, obs):
        """While carrying (states 5, 67, 68): fail into state 4 once bilateral
        finger contact has been gone for 30 ticks, instead of driving on empty
        and trying to place nothing. Returns True when the object was lost."""
        follower = getattr(self, "grasp_follower", None)
        if follower is None:
            return False
        self._lost_ticks = 0 if follower._contacts() else getattr(self, "_lost_ticks", 0) + 1
        if self._lost_ticks < 30:
            return False
        self.grasp_error = f"Lost the object in state {self.state} at {np.round(obs['object_pos'], 3).tolist()}"
        self.lost_in_state = self.state
        print(self.grasp_error)
        self.state = 4
        return True

    ## This function is called every time step.
    def get_action(self,env):
        obs = env._get_obs()
        if self.state in (5, 67, 68) and self._check_holding(obs):
            return self.action.copy()

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
            # Hold the lifted grasp briefly (confirms the object is secure), then
            # back away from the table before tucking the arm for travel.
            self._hold_ticks = getattr(self, "_hold_ticks", 0) + 1
            if hasattr(self, "grasp_follower"):
                self.action = self.grasp_follower.hold.copy()
                # Tighten the grip gradually over the hold (0.5 mm per tick).
                extra = min(self.carry_grip_extra, 0.0005 * self._hold_ticks)
                self._finger_hold = np.maximum(0.0, self.grasp_follower.hold[13:15] - extra)
                self.action[13:15] = self._finger_hold
            qpos = np.asarray(obs["qpos"], dtype=float)
            base_xy = np.array([qpos[0] - 2.0, qpos[1]])
            if getattr(self, "_retreat_away", None) is None:
                away = base_xy - np.asarray(obs["object_pos"])[:2]
                self._retreat_away = away / max(np.linalg.norm(away), 1e-6)
            away = self._retreat_away
            # Turn the fingers to face the retreat direction before the base moves.
            aligned = 1.0
            if self.align_grip_to_travel:
                i = self._wrist_roll_index
                self.action[i] = getattr(self, "_hold_roll", self.action[i])
                aligned = self._align_grip(self.action, away)
                self._hold_roll = self.action[i]
            if self._hold_ticks >= self.hold_before_transport and (aligned > 0.995 or self._hold_ticks > 200):
                # Retreat target in qpos coordinates (base x = qpos[0] - 2).
                self._retreat_goal = qpos[:2] + self.retreat_distance * away
                self.state = 5
            return self.action.copy()

        if self.state == 5:
            # Straight-line retreat with the arm and fingers frozen at the hold
            # pose, so the tuck motion cannot sweep the object into the table.
            qpos = np.asarray(obs["qpos"], dtype=float)
            error = self._retreat_goal - qpos[:2]
            step = error * min(1.0, self._ramped_speed(np.linalg.norm(error)) / max(np.linalg.norm(error), 1e-9))
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
                self._tuck_rate = self.tuck_rate   # slow tuck so the held object is not flung
                # The parking map leaves the tabletop out so the base can tuck
                # under it; for transport the whole table must be an obstacle.
                self.create_map(env, False)
                if hasattr(self, "_moving_pose_target"):
                    del self._moving_pose_target
                print("Retreat complete; navigating to goal with object.")
            return self.action.copy()
        # During grasp debugging, retain the failure hold target.
        if self.state == 4:
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
                factor = 1.0
                if self.align_grip_to_travel and getattr(self, "_travel_pose_complete", False) and norm > 1e-6:
                    factor = self._align_grip(action, delta / norm)
                speed = self._ramped_speed(norm, factor)
                if norm > speed:
                    action[:2] = qpos[:2] + delta * (speed / norm)
                # Keep squeezing the object; travel poses and navigation steps
                # otherwise command the fingers to their measured (unsqueezed) width.
                action[13:15] = finger_hold
                self.action = action.copy()
            return action

    def _ramped_speed(self, remaining, factor=1.0):
        """Base displacement for this tick while carrying: at most
        carry_speed * factor, changing by at most carry_accel per tick, and
        low enough to slow to a stop within `remaining` metres."""
        limit = min(self.carry_speed * factor, np.sqrt(2.0 * self.carry_accel * max(remaining, 0.0)) + self.carry_accel)
        previous = getattr(self, "_carry_v", 0.0)
        # The ramp only limits speeding up; slowing down follows `limit`,
        # which already falls off smoothly towards a stop.
        self._carry_v = float(min(limit, previous + self.carry_accel))
        return self._carry_v

    def _align_grip(self, action, direction):
        """Turn the wrist roll so the gripper's closing axis lies along the
        direction of travel, and return a speed factor (1 when aligned).

        The base controller kicks forward at the start of every step. With the
        closing axis along the travel direction the fingers take that kick as
        a normal force; across it, the object is held by friction alone and
        creeps along the pads. In the carry pose the gripper points straight
        up, so the wrist roll turns the closing axis about the vertical
        without moving anything else."""
        planner = self.grasp_planner
        state = p.getLinkState(planner.robot, planner.ee, computeForwardKinematics=True,
                               physicsClientId=planner.client)
        rot = np.asarray(p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
        closing = rot[:2, 1]
        if np.linalg.norm(closing) < 0.3:
            return 1.0   # closing axis nearly vertical: the roll cannot align it
        heading = np.arctan2(closing[1], closing[0])
        # The fingers are symmetric: align modulo pi, taking the short way.
        error = (np.arctan2(direction[1], direction[0]) - heading + np.pi / 2) % np.pi - np.pi / 2
        sign = 1.0 if rot[2, 0] >= 0 else -1.0   # roll axis (gripper x) up or down
        i = self._wrist_roll_index
        action[i] = action[i] + sign * float(np.clip(error, -self.grip_align_rate, self.grip_align_rate))
        # Wait for the fingers to come round before moving (start of the
        # route, sharp turns): a kick across the pads is what slides the
        # object out. Small errors only slow the base a little.
        return 0.0 if abs(error) > 0.3 else float(np.cos(error) ** 2)

    def reach_goal(self, obs):
        """State 68: put the held object on the goal sphere.

        1. back the base off along the arrival path (straight, slow);
        2. IK (planner's private torso+arm model) for a gripper pose that puts
           the object -- not the gripper -- at the goal, keeping the object's
           offset in the gripper frame fixed, then RRT-Connect from the carry
           pose to that solution with the object attached, checked against the
           maze walls. Interpolating straight in joint space from the carry
           pose swung the gripper sideways into the 1 m-wide corridor walls
           and knocked the object out;
        3. follow the path slowly with the fingers still squeezing.
        """
        qpos = np.asarray(obs["qpos"], dtype=float)
        action = self.action.copy()
        action[13:15] = self._finger_hold
        if self._reach_phase == "backoff":
            error = self._reach_base_goal - qpos[:2]
            step = error * min(1.0, self._ramped_speed(np.linalg.norm(error)) / max(np.linalg.norm(error), 1e-9))
            action[:2] = qpos[:2] + step
            if np.linalg.norm(error) < 0.03:
                action[:2] = qpos[:2]
                self._reach_base_hold = qpos[:3].copy()
                self._reach_attempts = 0
                self._reach_phase = "plan"
        elif self._reach_phase == "plan":
            # Hold still while planning; retry (the IK and RRT are randomised)
            # a few times before giving up and just holding the object.
            action[:3] = self._reach_base_hold
            self._reach_attempts += 1
            self._reach_path = self._solve_reach(obs)
            self._reach_index = 0
            if self._reach_path is not None:
                self._reach_phase = "arm"
            elif self._reach_attempts >= 3:
                self._reach_phase = "hold"
            print("Reach plan:", "found" if self._reach_path is not None else "failed",
                  "attempt", self._reach_attempts)
        elif self._reach_phase == "arm":
            idx = self.grasp_planner.motion_indices
            action[:3] = self._reach_base_hold
            target = self._reach_path[self._reach_index]
            error = target - qpos[idx]
            # Common scale keeps the commanded motion on the checked segment.
            limits = np.r_[0.002, np.full(len(idx) - 1, self.reach_rate)]
            scale = max(1.0, float(np.max(np.abs(error) / limits)))
            action[idx] = qpos[idx] + error / scale
            last = self._reach_index == len(self._reach_path) - 1
            if np.max(np.abs(error)) < (0.01 if last else 0.03):
                if last:
                    self._reach_phase = "hold"
                    print("Object placed at goal.")
                else:
                    self._reach_index += 1
        elif self._reach_phase == "hold" and getattr(self, "_reach_path", None) is not None:
            action[self.grasp_planner.motion_indices] = self._reach_path[-1]
        self.action = np.clip(action, self.action_lower_bound, self.action_upper_bound)
        return self.action.copy()

    def _held_object_validity(self, obs, attached, deadline):
        """Collision checking for torso+arm motion with the object in hand.

        Returns (valid, start, lower, upper): valid(arm) checks one torso+arm
        configuration with the object at `attached` (its pose in the gripper
        frame); start is the measured configuration and lower/upper the joint
        bounds, with continuous joints limited to +/- pi around the start.
        Call valid() inside planner._snapshot() only.
        """
        planner = self.grasp_planner
        env = self.env
        client = planner.client
        qpos = np.asarray(obs["qpos"], dtype=float)
        idx = planner.motion_indices
        fingers = qpos[planner.finger_indices]
        lower, upper = planner.lower[idx].copy(), planner.upper[idx].copy()
        # A joint held at its limit is measured a hair past it; plan from the
        # limit, or every configuration fails the joint-limit test.
        start = np.clip(qpos[idx], lower, upper)
        lower[planner.wrap] = np.maximum(lower[planner.wrap], start[planner.wrap] - np.pi)
        upper[planner.wrap] = np.minimum(upper[planner.wrap], start[planner.wrap] + np.pi)
        # Collision world: everything but the robot and the held object. The
        # goal marker has no collision response but getClosestPoints still
        # reports it. The held object moves rigidly with the gripper here, so
        # its distance to the palm and fingers cannot change along any path;
        # checking it against them rejects every configuration, the start
        # included, once the object has crept to within collision_margin of
        # the palm (soup can, top-down grasp: 0.3 mm on seed 7). The object is
        # still checked against the walls (_valid's payload check) and against
        # the rest of the robot (below).
        planner._configure_measured_limits(qpos)
        planner.obstacles = [
            b for b in (p.getBodyUniqueId(i, physicsClientId=client)
                        for i in range(p.getNumBodies(physicsClientId=client)))
            if b not in (planner.robot, env.object_id, getattr(env, "goal_marker_id", None))
        ]
        planner.support_contacts = {}
        gripper_links = {planner.ee, *planner.finger_ids}

        def valid(arm):
            if not planner._valid(planner._full(qpos, arm, fingers), touch=True,
                                  attached=attached, deadline=deadline):
                return False
            # _valid has placed the object at its attached pose.
            for contact in p.getClosestPoints(planner.robot, env.object_id,
                                              planner.config.collision_margin,
                                              physicsClientId=client):
                if contact[3] not in gripper_links:
                    planner._last_invalid = {
                        "reason": "payload_robot_collision",
                        "robot_link_name": planner.link_names.get(contact[3], contact[3]),
                        "distance": float(contact[8]),
                    }
                    return False
            return True

        return valid, start, lower, upper

    def _plan_tuck(self, obs, goal, seconds=30.0):
        """Collision-checked torso+arm path from the measured pose to `goal`
        (motion_indices order) with the object in hand, or None."""
        from custom_libraries.grasp import RRTConnect, edge_samples, planning_clock
        planner = self.grasp_planner
        client = planner.client
        deadline = planning_clock() + seconds
        state = p.getLinkState(planner.robot, planner.ee, computeForwardKinematics=True,
                               physicsClientId=client)
        object_pose = p.getBasePositionAndOrientation(self.env.object_id, physicsClientId=client)
        attached = p.multiplyTransforms(*p.invertTransform(state[4], state[5]), *object_pose)
        valid, start, lower, upper = self._held_object_validity(obs, attached, deadline)
        goal = np.asarray(goal, dtype=float).copy()
        wrap = planner.wrap
        goal[wrap] = start[wrap] + (goal[wrap] - start[wrap] + np.pi) % (2 * np.pi) - np.pi
        scale = planner.motion_scale
        with planner._snapshot():
            if not valid(start):
                print("Tuck: start state invalid", planner._last_invalid)
                return None
            if not valid(goal):
                print("Tuck: carry pose invalid", planner._last_invalid)
                return None
            if all(valid(m) for m in edge_samples(start, goal, planner.motion_resolution)):
                return np.array([goal])
            rrt = RRTConnect(lower / scale, upper / scale, lambda x: valid(x * scale),
                             resolution=float(np.min(planner.motion_resolution / scale)),
                             step_size=planner.config.rrt_step,
                             max_iterations=planner.config.rrt_iterations, rng=planner.rng)
            path = rrt.plan(start / scale, goal / scale, deadline=deadline)
            if path is None:
                print("Tuck: RRT found no path", planner._last_invalid)
                return None
            return np.asarray(path)[1:] * scale

    def _tuck_with_object(self, obs):
        """Move the arm into the carry pose with the object in hand, along a
        collision-checked path (the object is swung from a top-down grasp to
        fingers-up). Falls back to a straight joint-space move if no path is
        found. Sets moving_pose_ready when the carry pose is reached."""
        planner = self.grasp_planner
        idx = planner.motion_indices
        qpos = np.asarray(obs["qpos"], dtype=float)
        if getattr(self, "_tuck_path", None) is None:
            names = [planner.action_names[i] for i in idx]
            goal = np.array([self.travel_joint_targets[n] for n in names])
            path = self._plan_tuck(obs, goal)
            if path is None:
                wrap = planner.wrap
                goal[wrap] = qpos[idx][wrap] + (goal[wrap] - qpos[idx][wrap] + np.pi) % (2 * np.pi) - np.pi
                path = np.array([goal])
            print("Tuck path:", len(path), "waypoints")
            self._tuck_path, self._tuck_index = path, 0
            self._tuck_base = qpos[:3].copy()
        action = self.action.copy()
        action[:3] = self._tuck_base
        action[13:15] = self._finger_hold
        target = self._tuck_path[self._tuck_index]
        error = target - qpos[idx]
        # Common scale keeps the commanded motion on the checked segment.
        limits = np.r_[0.002, np.full(len(idx) - 1, self._tuck_rate)]
        scale = max(1.0, float(np.max(np.abs(error) / limits)))
        action[idx] = qpos[idx] + error / scale
        last = self._tuck_index == len(self._tuck_path) - 1
        self.moving_pose_ready = False
        if np.max(np.abs(error)) < 0.03:
            if not last:
                self._tuck_index += 1
            elif np.max(np.abs(np.asarray(obs["qvel"])[idx])) < 0.05:
                action[idx] = target
                self.moving_pose_ready = True
                self._tuck_path = None
        self.action = np.clip(action, self.action_lower_bound, self.action_upper_bound)
        return self.action.copy()

    def _solve_reach(self, obs, seconds=60.0):
        """Return a collision-checked torso+arm path to the place pose, or None.

        `seconds` is CPU time; planning costs no simulation steps, so the
        budget is generous (plans measured at 2.5-7 CPU-s for the soup can).
        """
        from scipy.spatial.transform import Rotation
        from custom_libraries.grasp import _ManipulatorIK, RRTConnect, edge_samples, planning_clock
        planner = self.grasp_planner
        env = self.env
        client = planner.client
        qpos = np.asarray(obs["qpos"], dtype=float)
        deadline = planning_clock() + seconds
        state = p.getLinkState(planner.robot, planner.ee, computeForwardKinematics=True,
                               physicsClientId=client)
        g_pos, g_rot = np.asarray(state[4]), Rotation.from_quat(state[5])
        object_pose = p.getBasePositionAndOrientation(env.object_id, physicsClientId=client)
        inv = p.invertTransform(state[4], state[5])
        attached = p.multiplyTransforms(*inv, *object_pose)
        offset_local = g_rot.inv().apply(np.asarray(obs["object_pos"]) - g_pos)
        goal = np.asarray(obs["goal_pos"], dtype=float)
        valid, start, lower, upper = self._held_object_validity(obs, attached, deadline)
        scale = planner.motion_scale

        ik = _ManipulatorIK(planner, qpos)
        try:
            with planner._snapshot():
                if not valid(start):
                    print("Reach: start state invalid", planner._last_invalid)
                # Current gripper orientation first, then level/forward variants.
                rotations = [g_rot] + [Rotation.from_euler("xyz", [r, 0, y])
                                       for y in (0.0, 0.5, -0.5)
                                       for r in (0.0, np.pi/2, -np.pi/2)]
                # The task only needs the object within 10 cm of the goal, so
                # aim at the centre first, then at points 5 cm away from it.
                aims = [np.zeros(3)] + [0.05 * np.asarray(d, float) for d in
                                        ((-1, 0, 0), (0, 0, 1), (0, 1, 0), (0, -1, 0), (0, 0, -1))]
                candidates = [(rot, aim) for aim in aims for rot in rotations]
                # Pass 1: IK for every candidate, taking the first one whose
                # straight joint-space move is collision free (cheap). Running
                # a 6 s RRT for each candidate as it came used up the budget
                # on 2-3 hard candidates before any easy one was tried (soup
                # can, seed 42: 2 of 5 plans failed that way).
                needs_rrt = []
                for rot, aim in candidates:
                    if planning_clock() >= deadline:
                        break
                    target = goal + aim - rot.apply(offset_local)
                    q = ik.solve(target, rot.as_quat(), start, lower, upper,
                                 min(deadline, planning_clock() + 2.0), restarts=12)
                    if q is None:
                        print("Reach: IK failed for orientation", np.round(rot.as_euler("xyz"), 2))
                        continue
                    if not valid(q):
                        print("Reach: goal pose invalid", planner._last_invalid)
                        continue
                    if all(valid(m) for m in edge_samples(start, q, planner.motion_resolution)):
                        return np.array([q])
                    needs_rrt.append(q)
                # Pass 2: RRT to the IK solutions nearest the start first.
                needs_rrt.sort(key=lambda q: float(np.linalg.norm((q - start) / scale)))
                for q in needs_rrt:
                    if planning_clock() >= deadline:
                        break
                    rrt = RRTConnect(lower / scale, upper / scale,
                                     lambda x: valid(x * scale),
                                     resolution=float(np.min(planner.motion_resolution / scale)),
                                     step_size=planner.config.rrt_step,
                                     max_iterations=planner.config.rrt_iterations,
                                     rng=planner.rng)
                    path = rrt.plan(start / scale, q / scale,
                                    deadline=min(deadline, planning_clock() + 6.0))
                    if path is not None:
                        return np.asarray(path)[1:] * scale
                    print("Reach: RRT found no path", planner._last_invalid)
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
        if hasattr(self, "_tuck_rate") and getattr(self, "_finger_hold", None) is not None:
            return self._tuck_with_object(obs)
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
