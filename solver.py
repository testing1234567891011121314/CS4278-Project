from collections import deque

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
        # Project walls at the rest pose (before the arm ever extends):
        # projecting mid-carry with the arm out gives a ~1 m silhouette that
        # blocks every corridor.
        self.wall_blocked = build_wall_collision_grid(
            env, self.wall_occupancy, self.cell_size, safety_margin=0.05)
        self.inflated_occupancy = self.create_navigable_map() # returns inflated occupancy grid
        self.carry_occupancy = self.create_carry_map()
        
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
        self.latched_reposition_goal = None

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
            try:
                decision = self.navigate_grasp.update(obs, self.inflated_occupancy, self.convert_to_grid, self.convert_to_world, self.cell_size)
            except RuntimeError as exc:
                print("No navigation goal remains:", exc)
                self.state = 4
                return np.asarray(obs["qpos"]).copy()
            goal_world = decision["navigation_goal"]
            base_xy = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])

            print("solver state:", self.state, "decision:", decision["state"])

            standing_cell = tuple(np.round(
                np.asarray(self.convert_to_world(
                    self.convert_to_grid(base_xy))), 3))

            # Backoff: yaw/creep got wedged against the table at this cell.
            # Drive straight back from the object, then re-park from the new
            # pose (retrying from the wedged spot re-latches a cell right
            # next to it and re-wedges -- 47 abandons without moving).
            backoff = getattr(self, "_grasp_backoff", None)
            if backoff is not None:
                # Minimum 12 ticks: at the moment the yaw/creep jam is
                # detected the base is by definition stationary, so a
                # settle check alone ended the backoff on tick 0 and the
                # retry re-parked to the same wedged spot forever.
                if (self._backoff_ticks >= 12
                        and float(np.linalg.norm(obs["qvel"][:2])) < 0.01) \
                        or self._backoff_ticks >= 40:
                    self._grasp_backoff = None
                    self._retry_after_grasp_failure()
                    return self.action.copy()
                self._backoff_ticks += 1
                away = base_xy - np.asarray(obs["object_pos"], float)[:2]
                away /= max(float(np.linalg.norm(away)), 1e-6)
                self.action = np.asarray(obs["qpos"], float).copy()
                self.action[:2] = obs["qpos"][:2] + np.clip(0.02 * away, -0.05, 0.05)
                return self.action.copy()

            if standing_cell in self.navigate_grasp.failed_goals:
                # The base stands on a cell a plan already proved unable to
                # grasp from; ignore a GRASP decision here and re-park.
                if (self.latched_reposition_goal is not None
                        and tuple(np.round(np.asarray(
                            self.latched_reposition_goal), 3))
                        in self.navigate_grasp.failed_goals):
                    self.latched_reposition_goal = None
                if self.latched_reposition_goal is None:
                    try:
                        self.latched_reposition_goal = self.navigate_grasp.nearest_free_goal(
                            np.asarray(obs["object_pos"], float)[:2],
                            self.inflated_occupancy, self.convert_to_grid,
                            self.convert_to_world, self.cell_size,
                            robot_xy=base_xy)
                    except RuntimeError as exc:
                        print("No navigation goal remains:", exc)
                        self.state = 4
                        return np.asarray(obs["qpos"]).copy()
                goal_world = self.latched_reposition_goal
                decision = {"state": "NAVIGATE_TO_GRASP",
                            "navigation_goal": goal_world}

            arrived = (
                goal_world is not None
                and np.linalg.norm(base_xy - goal_world) < 0.10
            )
            # Don't hand off to planning unless the base faces the object:
            # at the park cell the grasp axis must sit near the forward
            # half-plane, otherwise the arm approaches from a side angle
            # and the transit/IK basins collapse (F-3 feeds F-1/F-4).
            if arrived or decision["state"] == "GRASP":
                object_xy = np.asarray(obs["object_pos"], float)[:2]
                delta = object_xy - base_xy  # base world frame (qpos x carries -2.0 offset)
                bearing = np.arctan2(delta[1], delta[0])
                err = (bearing - obs["qpos"][2] + np.pi) % (2 * np.pi) - np.pi
                if abs(err) > 0.35:
                    self.forward = 0.0
                    self.turn = 0.0
                    self.yaw = np.clip(err / self.action_scale[2], -2.0, 2.0)
                    # Anchor xy too: without this the branch inherited
                    # self.action[:2] from the previous branch forever and
                    # the stale target dragged the base 0.26 m into the
                    # table edge (r002 pinned at (-0.34,0.635) 2000 ticks).
                    self.action[:2] = obs["qpos"][:2]
                    self.action[2] = obs["qpos"][2]  # anchor: accumulate toward bearing
                    unit_action = [self.forward, self.turn, self.yaw,
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                    self.action = np.clip(
                        self.action + self.action_scale * np.asarray(unit_action),
                        self.action_lower_bound, self.action_upper_bound)
                    # Net yaw progress over a window, not |qvel|: a base
                    # pinned laterally vibrates its yaw joint and fooled the
                    # velocity check into an eternal spin (r002).
                    yhist = getattr(self, "_yaw_hist", None)
                    if yhist is None:
                        yhist = self._yaw_hist = []
                    yhist.append(float(obs["qpos"][2]))
                    if len(yhist) > 60:
                        yhist.pop(0)
                    if len(yhist) >= 60 and abs(yhist[0] - yhist[-1]) > 0.15:
                        self._yaw_stuck = 0
                    else:
                        self._yaw_stuck = getattr(self, "_yaw_stuck", 0) + 1
                        if self._yaw_stuck > 120:
                            self._yaw_stuck = 0
                            self._yaw_hist = None
                            print("Yaw cannot close: backing off.")
                            self._grasp_backoff = True
                            self._backoff_ticks = 0
                    return self.action.copy()
                self.yaw = 0.0
                # With the torso lifted the arm plans at 0.65-0.80 m
                # (probe_max_park: clean at 0.70/0.75, transit-capped at
                # 0.80, closure-heavy <0.65); cell quantization + arrival
                # slack can leave the base outside, so trim radially first.
                stand_off = float(np.linalg.norm(delta))
                if not 0.66 <= stand_off <= 0.80:
                    self.forward = 0.0
                    self.turn = 0.0
                    creep = float(np.clip(stand_off - 0.72, -0.15, 0.15))
                    ux, uy = delta / max(stand_off, 1e-6)
                    self.action[:2] = obs["qpos"][:2]  # anchor (navigate() convention)
                    unit_action = [ux * creep / self.action_scale[0],
                                   uy * creep / self.action_scale[1],
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                    self.action = np.clip(
                        self.action + self.action_scale * np.asarray(unit_action),
                        self.action_lower_bound, self.action_upper_bound)
                    # The base can press against the table edge here and
                    # never close the stand-off gap (the post-retry park at
                    # 0.83 m sat against the table for 900 steps). If the
                    # base isn't actually moving, blacklist the cell and
                    # re-park.
                    # Progress-based, not velocity-based: when the base is
                    # blocked it can enter a 2-step limit cycle (recomputed
                    # targets flip while the arm servo lags), which keeps
                    # qvel high so a velocity check never fires. Require the
                    # stand-off error to actually shrink over 60 ticks.
                    hist = getattr(self, "_creep_hist", None)
                    if hist is None:
                        hist = self._creep_hist = []
                    hist.append(abs(stand_off - 0.72))
                    if len(hist) > 60:
                        hist.pop(0)
                    if len(hist) >= 60 and hist[0] - hist[-1] > 0.015:
                        self._creep_stuck = 0
                    else:
                        self._creep_stuck = getattr(self, "_creep_stuck", 0) + 1
                        if self._creep_stuck > 80:
                            self._creep_stuck = 0
                            self._creep_hist = None
                            print("Creep cannot close the stand-off: "
                                  "backing off.")
                            self._grasp_backoff = True
                            self._backoff_ticks = 0
                    return self.action.copy()
            if decision["state"] == "GRASP" or arrived:
                # A GRASP decision can fire on the spawn cell before the arm
                # ever reached the travel pose; planning from the drooped
                # post-spawn arm puts the open fingers inside the table
                # (every candidate dies at the opening check). Raise the
                # torso and fold the arm first, then hand off.
                if not getattr(self, "_travel_pose_complete", False):
                    pose_action = self.moving_pose(obs)
                    self._pose_wait = getattr(self, "_pose_wait", 0) + 1
                    if self._pose_wait > 400:
                        self._pose_wait = 0
                        print("Travel pose unreachable here: backing off.")
                        self._grasp_backoff = True
                        self._backoff_ticks = 0
                    if self.moving_pose_ready:
                        self._travel_pose_complete = True
                    return pose_action
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
                return self._settle_hold(obs)

            goal = self.convert_to_grid(decision["navigation_goal"])
            return self.navigate(env, goal)

        if self.state == 1:
            print("solver state:", self.state)
            from pprint import pprint
            import time

            # Hold the commanded travel pose. Releasing the arm to its
            # measured pose lets gravity fold it into the table before the
            # plan runs, so every candidate dies at the opening check.
            self.action = self._settle_hold(obs)
            object_pos = np.asarray(obs["object_pos"], dtype=float)

            previous = getattr(self, "_grasp_last_object_pos", None)
            self._grasp_last_object_pos = object_pos.copy()

            target = getattr(self, "_moving_pose_target", None)
            arm_ok = (target is None
                      or np.max(np.abs(target[6:13] - obs["qpos"][6:13])) < 0.05)
            settled = (
                previous is not None
                and np.linalg.norm(object_pos - previous) < 0.001
                and np.max(np.abs(obs["qvel"])) < 0.05
                and arm_ok
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
                if not self._retry_after_grasp_failure():
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
                print("Grasp stopped:", self.grasp_error)
                self.grasp_follower = None
                if not self._retry_after_grasp_failure():
                    self.state = 4
            return self.action.copy()

        if self.state == 3:
            return self._carry_action(obs)
        
        if self.state == 4:
            print("solver state:", self.state)
            # Return numeric absolute targets, never an exception class.
            self.action = np.clip(
                np.asarray(self.action, dtype=float),
                self.action_lower_bound, self.action_upper_bound,
            )
            return self.action.copy()

    def _settle_hold(self, obs):
        """Hold the commanded travel pose (base anchored) while settling.

        Holding the *measured* pose instead lets the arm free-fall into the
        table under gravity, so every candidate then dies at the opening
        collision check with fingers inside the tabletop.
        """
        target = getattr(self, "_moving_pose_target", None)
        if target is None:
            return np.asarray(obs["qpos"], dtype=float).copy()
        hold = target.copy()
        hold[:3] = obs["qpos"][:3]  # keep the base where it is
        return np.clip(hold, self.action_lower_bound, self.action_upper_bound)

    def _carry_action(self, obs):
        """Payload transport to the goal (journal priority 5).

        Carry at FULL lift height: the payload hangs at z ~0.70-0.79,
        clearing the tabletop top (0.626) by ~11 cm, so the base can pass
        the table normally -- the map already blocks the legs (they start
        at z=0) and the 2 m maze walls. Lowering the payload instead puts
        the arm against the tabletop underside (0.574), which the arm
        cannot fold below (measured minimum z ~0.68), so a low carry is
        physically impossible. Then servo on the object and raise via
        torso in the final metre. Never reopens the gripper.
        """
        print("solver state:", self.state)
        q = np.asarray(obs["qpos"], float)
        goal = np.asarray(self.env.goal_pos, float)
        base_xy = np.array([q[0] - 2.0, q[1]])
        obj = np.asarray(obs["object_pos"], float)
        hold = self.grasp_follower.hold.copy() \
            if getattr(self, "grasp_follower", None) is not None else q.copy()

        touching = bool(p.getContactPoints(
            bodyA=self.env.object_id, bodyB=self.env.robot_id,
            physicsClientId=self.env.pb_physics_client))
        self._carry_no_contact = 0 if touching \
            else getattr(self, "_carry_no_contact", 0) + 1

        # Scrape guard: every shed happened with the apple pressed against
        # a world obstacle (it hangs ~0.7 m off the base axis, outside the
        # base-disk inflation the carry map knows about). Wall friction on
        # the pressed side peels the payload from the pads. Freeze XY while
        # the payload grinds an obstacle so the peel never accumulates.
        # Legacy contact tuples put an id-like value in c[2], not the
        # "other body" slot, so the old c[2] != robot_id filter screened out
        # everything and the guard read 0.0 while the apple ground an
        # obstacle at 736 N. Per-body queries are unambiguous.
        scrape = 0.0
        for w in range(p.getNumBodies(physicsClientId=self.env.pb_physics_client)):
            if w in (self.env.object_id, self.env.robot_id):
                continue
            for c in p.getContactPoints(bodyA=self.env.object_id, bodyB=w,
                                        physicsClientId=self.env.pb_physics_client):
                if c[9] > scrape:
                    scrape = c[9]
        self._carry_scrape = scrape

        # Payload hangs ~0.7 m ahead; anything beyond ~1.2 m was dropped
        # (on the tabletop at z~0.66, above the fall threshold). A grip that
        # has held no contact for 10 ticks shed the payload -- waiting for
        # the z/distance thresholds meant dragging the apple on the table
        # for 900 steps after every slip.
        if (obj[2] < 0.30 or float(np.linalg.norm(obj[:2] - base_xy)) > 1.2
                or self._carry_no_contact > 10):
            print("Carry: object fell; returning to the grasp loop.")
            try:
                ct = [(c[2], c[4], round(c[8], 5)) for c in
                      p.getContactPoints(bodyA=self.env.object_id,
                                         physicsClientId=self.env.pb_physics_client)]
                print("  fall obj_z=", round(float(obj[2]), 4),
                      "obj=", np.round(obj[:2], 3),
                      "base=", np.round(base_xy, 3),
                      "rf=", round(float(q[13]), 5),
                      "lf=", round(float(q[14]), 5),
                      "ct=", ct[:12], flush=True)
            except Exception:
                pass
            self.grasp_follower = None
            self.navigate_grasp.grasp = None
            self.carry_path = None
            self.path = None
            self.curved_nav.reset()
            self.action[13:15] = self.action_upper_bound[13:15]
            for attr in ("_carry_stage", "_carry_torso_cmd", "_carry_lift_cmd",
                         "_carry_stuck", "_carry_replans", "_carry_servo",
                         "_carry_servo_hold",
                         "_carry_escape", "_carry_cmd_xy", "_carry_had_contact",
                         "_carry_grip_extra", "_carry_z_ref",
                         "_payload_pruned"):
                if hasattr(self, attr):
                    delattr(self, attr)
            self.state = 0
            return q.copy()

        if not hasattr(self, "_carry_stage"):
            self._carry_stage = "travel"
            self._carry_stuck = 0
            self._carry_replans = 0
            self._carry_escape = None
            self._carry_had_contact = True
            self._carry_tail = []
            self._carry_tick = 0
            self._carry_pos_hist = []
            # The base map only inflates for the base disk, but the payload
            # rides ~0.75 m ahead and ~0.09 m off-axis -- the corridors are
            # 1 m wide and 0.9 m of clearance is gone, so A* happily plans
            # centrelines that drag the apple along the wall (shed at base
            # (1.2,3.9) with scrape force below the 20 N guard, and every
            # contact-LOST tail showed a slow force decay against something).
            # Prune any cell whose payload disc (radius ~0.10 m at the
            # object's pose if the base centre stood there) overlaps an
            # obstacle. Once per carry; the map persists across sheds.
            if not getattr(self, "_payload_pruned", False):
                base_map = self.create_carry_map() != 0
                cell = float(self.cell_size)
                radius = 0.11  # apple radius ~0.07 + pad slop
                # Travel freezes yaw, so the payload sits at a fixed offset
                # (ahead ~0.76, lat ~0.06) in the base frame; a base centre
                # is only safe if the payload disc at that offset is clear.
                ahead_o, lat_o = 0.76, 0.06
                yaw = float(q[2])
                off = np.array([ahead_o * np.cos(yaw) - lat_o * np.sin(yaw),
                                ahead_o * np.sin(yaw) + lat_o * np.cos(yaw)])
                # Grid axes: row = -y, col = +x (convert_to_grid).
                off_cell = (int(round(off[1] / cell)),
                            int(round(off[0] / cell)))
                # Pad must cover both the disc radius AND the payload
                # offset from the base centre (ahead ~8 cells) or the
                # shifted window runs off the padded grid.
                pad = int((radius + abs(off[0]) + abs(off[1])) / cell) + 1
                big = np.zeros((base_map.shape[0] + 2 * pad,
                                base_map.shape[1] + 2 * pad), dtype=bool)
                big[pad:pad + base_map.shape[0],
                    pad:pad + base_map.shape[1]] = base_map
                blocked = base_map.copy()
                for dr in range(-pad, pad + 1):
                    for dc in range(-pad, pad + 1):
                        if (dr * dr + dc * dc) * cell * cell > radius ** 2:
                            continue
                        shifted = big[pad + dr + off_cell[0]:
                                      pad + dr + off_cell[0]
                                      + base_map.shape[0],
                                      pad + dc + off_cell[1]:
                                      pad + dc + off_cell[1]
                                      + base_map.shape[1]]
                        blocked |= shifted
                # Keep the cell under the base free or A* can never start.
                start_cell = tuple(int(v) for v in
                                   self.convert_to_grid(base_xy))
                blocked[start_cell] = False
                # Keep the raw base map for the escape/BFS branches; A* and
                # the navigator plan on the payload-pruned map.
                self.carry_plan_map = blocked.astype(np.uint8)
                self._payload_pruned = True
                print("Carry: payload-pruned map;",
                      int(blocked.sum()), "blocked cells", flush=True)
            print("Carry: stage=travel (full height)", flush=True)

        had = getattr(self, "_carry_had_contact", True)
        touching = bool(p.getContactPoints(
            bodyA=self.env.object_id, bodyB=self.env.robot_id,
            physicsClientId=self.env.pb_physics_client))
        # Ring buffer of the last 20 grip states; dumped on shed so the peel
        # mechanism is visible tick-by-tick (100-tick telemetry showed the
        # apple stable at 800 N and gone 50 ticks later with no cause).
        _fwd = np.array([np.cos(q[2]), np.sin(q[2])])
        _left = np.array([-np.sin(q[2]), np.cos(q[2])])
        _g = p.getContactPoints(bodyA=self.env.object_id,
                                bodyB=self.env.robot_id,
                                physicsClientId=self.env.pb_physics_client)
        _maxF = max((c[9] for c in _g if c[4] in (21, 22)), default=0.0)
        _d = obj[:2] - base_xy
        self._carry_tail = (getattr(self, "_carry_tail", [])
                            + [(round(_maxF, 1),
                                round(float(_d @ _left), 3),
                                round(float(_d @ _fwd), 3),
                                round(float(obj[2]), 3),
                                round(getattr(self, "_carry_scrape", 0.0), 1))])[-20:]
        if had and not touching:
            print(f"Carry: contact LOST stage={self._carry_stage} "
                  f"obj=({obj[0]:.3f},{obj[1]:.3f},{obj[2]:.3f}) "
                  f"base=({base_xy[0]:.3f},{base_xy[1]:.3f}) "
                  f"yaw={q[2]:.2f} rf={q[13]:.5f} lf={q[14]:.5f} "
                  f"servo={getattr(self, '_carry_servo', False)}", flush=True)
            for f_, l_, a_, z_, s_ in getattr(self, "_carry_tail", []):
                print(f"   tail: F={f_:6.1f} lat={l_:+.3f} "
                      f"ahead={a_:.3f} z={z_:.3f} scrapeF={s_:.1f}",
                      flush=True)
        self._carry_had_contact = touching

        action = hold.copy()
        # Clamp deeper than the grasp plan held: 6/8 mm crush the flat pads
        # against the sphere and squirt it out sideways (measured pads at
        # 28 mm, lat -0.21 m at first telemetry); 3 mm holds centred at
        # ~400 N. Adaptive: every shed tail showed the fingertip normal
        # force decaying smoothly to zero over ~20 ticks while the payload
        # geometry stayed frozen -- a slow grip relaxation. Deepen 0.5 mm
        # per tick (cap 6 mm, reached only after ~6 sustained ticks below
        # 150 N so a transient dip cannot crush-and-squirt) whenever the
        # grip is alive but weak.
        # RATCHET: once deepened, never relax mid-carry. An earlier version
        # reset the clamp at F >= 150 N, which chattered 3<->4.5 mm and saw
        # the force sawtooth 195/90/187/85 while the pads pumped the apple
        # out of the grip. Trigger high (250 N) so the decay is caught
        # while there is still grip to hold, and only ever deepen.
        grip_extra = getattr(self, "_carry_grip_extra", 0.003)
        if 0.0 < _maxF < 250.0:
            grip_extra = min(grip_extra + 0.00025, 0.006)
        elif _maxF > 650.0:
            # Over-crush shed: grip rode 1200-1400 N once and 555-711 N
            # again -- in both the pads squirted the sphere sideways (lat
            # crept +0.048 -> +0.059, z sagged 0.013 m) and then F
            # collapsed. Every shed tail started its trouble above ~600 N
            # while every carried segment held below ~510 N, so the relax
            # threshold sits at 650 with a 250-650 N deadband: the two
            # rules can never pump against each other.
            grip_extra = max(grip_extra - 0.0005, 0.0)
        self._carry_grip_extra = grip_extra
        action[13:15] = np.clip(
            action[13:15] - grip_extra,
            self.action_lower_bound[13:15], self.action_upper_bound[13:15])

        # Grip telemetry: max fingertip normal force + lateral offset of the
        # payload from the base forward axis. Distinguishes a forceless grip
        # (slip) from a forced grip held off-centre (wringing).
        self._carry_tele = getattr(self, "_carry_tele", 0) + 1
        if self._carry_tele % 100 == 0:
            forces = [c[9] for c in p.getContactPoints(
                bodyA=self.env.object_id, bodyB=self.env.robot_id,
                physicsClientId=self.env.pb_physics_client)
                if c[4] in (21, 22)]
            fwd = np.array([np.cos(q[2]), np.sin(q[2])])
            left = np.array([-np.sin(q[2]), np.cos(q[2])])
            lat = float((obj[:2] - base_xy) @ left)
            ahead = float((obj[:2] - base_xy) @ fwd)
            print(f"Carry grip step~{self._carry_tele} "
                  f"maxF={max(forces) if forces else 0.0:.1f} "
                  f"n={len(forces)} lat={lat:.3f} ahead={ahead:.3f} "
                  f"obj_z={obj[2]:.3f}", flush=True)
        action[:3] = q[:3]
        # Command the commanded torso, not the measured one: re-anchoring to
        # the measured value each step lets the torso drift down ~14 cm over a
        # carry, and the payload's inertial loads then pry it out of the pads.
        action[3] = hold[3]

        # During travel the yaw follows the path tangent so the extended
        # arm leads down the corridor instead of cutting corners into the
        # walls; the final servo faces the goal to place the payload.
        heading = np.arctan2(goal[1] - q[1], goal[0] - base_xy[0])

        d_xy = float(np.linalg.norm(obj[:2] - goal[:2]))
        if self._carry_stage == "travel" and d_xy <= 1.0:
            self._carry_stage = "final"
            print("Carry: stage=final", flush=True)

        if self._carry_stage == "travel" and not getattr(self, "_carry_servo", False):
            if getattr(self, "carry_occupancy", None) is None:
                self.carry_occupancy = self.create_carry_map()
            start_cell = self.convert_to_grid(base_xy)
            # Escape if blocked in EITHER map: the pruned map force-frees
            # the start cell (A* must be able to start), so the raw map's
            # table footprint is what catches a base standing on the table.
            if (self.carry_plan_map[tuple(start_cell)] != 0
                    or self.carry_occupancy[tuple(start_cell)] != 0):
                # Grasp pose is unsafe for the payload (base inside the
                # inflated table footprint, and/or the apple hanging over
                # the table edge into its obstacle disc): drive out to the
                # nearest payload-clear cell before A* can start.
                if getattr(self, "_carry_escape", None) is None:
                    # Free in BOTH maps: the pruned map force-frees the
                    # start cell (A* must start somewhere), so choosing
                    # from it alone lets the escape target be the table
                    # cell the base is standing on -- target == base, the
                    # escape never terminates (2960 frozen ticks).
                    free = np.argwhere((self.carry_plan_map == 0)
                                       & (self.carry_occupancy == 0))
                    esc = free[np.argmin(
                        np.linalg.norm(free - np.asarray(start_cell), axis=1))]
                    self._carry_escape = self.convert_to_world(tuple(esc))
                    print("Carry: escaping table footprint toward",
                          np.round(self._carry_escape, 2), flush=True)
                step_xy = self._carry_escape - base_xy
                if getattr(self, "_carry_scrape", 0.0) > 20.0:
                    # The payload grinds the table edge on the way out --
                    # wall friction peels it from the pads no matter how
                    # gently we drive. Set it down instead: open the fingers
                    # here, back off from the obstacle, and the shed-abort
                    # below returns to the grasp loop for a fresh grasp from
                    # a better stand-off.
                    print("Carry: escape scrapes payload; setting it down "
                          "and backing off.", flush=True)
                    self.action = np.asarray(q, float).copy()
                    self.action[13:15] = self.action_upper_bound[13:15]
                    away = base_xy - obj[:2]
                    away /= max(float(np.linalg.norm(away)), 1e-6)
                    self.action[:2] = q[:2] + np.clip(0.02 * away, -0.05, 0.05)
                    return np.clip(self.action, self.action_lower_bound,
                                   self.action_upper_bound)
                # Same acceleration limit as travel, and NO yaw steer while
                # escaping: the base is omnidirectional, and turning with a
                # 0.7 m payload swings it off the pads like a pendulum
                # (this exact escape shed the apple at the table edge).
                desired = base_xy + step_xy
                if not hasattr(self, "_carry_cmd_xy"):
                    self._carry_cmd_xy = base_xy.copy()
                # 2 cm/step: a straight drive has no sawtooth to damp (the
                # travel limiter exists for the path-projection re-fit), and
                # 1 cm/step could not clear the inflated footprint inside
                # the step budget.
                move = desired - self._carry_cmd_xy
                dist = float(np.linalg.norm(move))
                if dist > 0.02:
                    self._carry_cmd_xy += move * (0.02 / dist)
                else:
                    self._carry_cmd_xy = desired
                cmd = np.clip(self._carry_cmd_xy - base_xy, -0.05, 0.05)
                action[0] = q[0] + float(cmd[0])
                action[1] = q[1] + float(cmd[1])
                self.action = np.clip(action, self.action_lower_bound,
                                      self.action_upper_bound)
                return self.action.copy()

            if getattr(self, "carry_path", None) is None:
                dirv = base_xy - goal[:2]
                d = max(float(np.linalg.norm(dirv)), 1e-6)
                stand = goal[:2] + 0.90 * dirv / d  # gripper reaches ~0.8 forward
                stand_cell = np.asarray(self.convert_to_grid(stand), dtype=int)
                # Plan on the raw base map: with yaw frozen during travel
                # the payload offset (ahead 0.76) makes every corridor
                # perpendicular to the heading impassable in a
                # payload-pruned map (A* returned length 1 on every replan
                # and the servo fallback sheared the apple off the pads at
                # ~700 N). The raw map's centreline grazes walls but has
                # carried 6 m; the scrape guard and stuck-replan handle
                # grazes. The pruned map still gates the escape from the
                # grasp pose, where the payload hangs over the table edge.
                free = np.argwhere(self.carry_occupancy == 0)
                if self.carry_occupancy[tuple(stand_cell)] != 0:
                    # Inflation near the goal marker blocks the ideal cell;
                    # the fine servo closes the last metre, so any free cell
                    # near the stand position works.
                    stand_cell = free[np.argmin(
                        np.linalg.norm(free - stand_cell, axis=1))]
                self.carry_path = a_star(
                    start_cell,
                    tuple(stand_cell),
                    self.carry_occupancy)
                if len(self.carry_path) < 2:
                    # Table/wall inflation cuts the loaded region off from
                    # the stand cell. Servoing across that gap pushes the
                    # cart into the obstacle; instead drive to the closest
                    # reachable free cell and servo from there.
                    reach = np.zeros_like(self.carry_occupancy, dtype=bool)
                    start = tuple(int(v) for v in start_cell)
                    if self.carry_occupancy[start] == 0:
                        queue = deque([start])
                        reach[start] = True
                        while queue:
                            r, c = queue.popleft()
                            for dr in (-1, 0, 1):
                                for dc in (-1, 0, 1):
                                    nr, nc = r + dr, c + dc
                                    if (0 <= nr < reach.shape[0]
                                            and 0 <= nc < reach.shape[1]
                                            and not reach[nr, nc]
                                            and self.carry_occupancy[nr, nc] == 0):
                                        reach[nr, nc] = True
                                        queue.append((nr, nc))
                    cand = free[reach[free[:, 0], free[:, 1]]]
                    if len(cand) == 0:
                        self.carry_path = []
                    else:
                        tgt = cand[np.argmin(
                            np.linalg.norm(cand - np.asarray(stand_cell),
                                           axis=1))]
                        print("Carry: stand cell unreachable; targeting",
                              np.round(self.convert_to_world(tuple(tgt)), 2),
                              flush=True)
                        self.carry_path = a_star(start_cell, tuple(tgt),
                                                 self.carry_occupancy)
                self.waypoint_index = 0
                self.curved_nav.reset()
                print("Carry path length:",
                      None if self.carry_path is None else len(self.carry_path),
                      flush=True)
            if not self.carry_path:
                print("Carry: no A* path to the goal cell; servoing instead.",
                      flush=True)
                self._carry_servo = True
            else:
                # Net displacement over a ~60-tick window, not |qvel|: a
                # base pinned against an obstacle vibrates at 1-4 cm/s
                # (qvel reset each substep) and fooled the speed detector
                # into 1700 silent ticks (r001, world (-1.91,-0.01)).
                self._carry_tick = getattr(self, "_carry_tick", 0) + 1
                hist = getattr(self, "_carry_pos_hist", None) or []
                if not hist or self._carry_tick % 10 == 0:
                    hist = hist[-6:] + [base_xy.copy()]
                    self._carry_pos_hist = hist
                moved = max(
                    (float(np.linalg.norm(base_xy - h)) for h in hist),
                    default=0.0)
                self._carry_stuck = self._carry_stuck + 1 if moved < 0.03 else 0
                if self._carry_stuck > 60:
                    # Physically jammed though the map says free (arm or
                    # payload clipping an edge the base map does not see):
                    # replan from here; after three replans, drive straight
                    # at the goal to slide off the obstruction.
                    self._carry_stuck = 0
                    self._carry_replans += 1
                    jam = sorted({(c[1], c[2], c[3], c[4],
                                   round(c[9], 1))
                                  for body in (0, 2, 3, 4, 5)
                                  if body != self.env.object_id
                                  for c in p.getContactPoints(
                                      bodyA=self.env.robot_id, bodyB=body,
                                      physicsClientId=self.env.pb_physics_client)
                                  if c[9] > 1.0}, key=lambda t: -t[4])
                    print(f"Carry: stuck at ({base_xy[0]:.2f},{base_xy[1]:.2f})"
                          f" replan {self._carry_replans} jam contacts "
                          f"(bodyA,bodyB,lkA,lkB,F)={jam[:10]}", flush=True)
                    ls = getattr(self, "_carry_last_step", None)
                    print("   stuck telemetry qvel=",
                          np.round(np.asarray(obs["qvel"], float)[:2], 3),
                          " last_step=",
                          None if ls is None else np.round(ls, 3),
                          " cmdxy=", np.round(getattr(self, "_carry_cmd_xy",
                                                      base_xy), 3),
                          " nav=", self.curved_nav.status,
                          " wp=", self.waypoint_index,
                          " pathlen=",
                          None if self.carry_path is None
                          else len(self.carry_path), flush=True)
                    ow = sorted(
                        {(c[2], c[5], round(c[9], 1),
                          tuple(np.round(c[6], 2)))
                         for c in p.getContactPoints(
                             bodyA=self.env.object_id,
                             physicsClientId=self.env.pb_physics_client)
                         if c[9] > 0.1}, key=lambda t: -t[2])
                    print("   obj-world contacts:", ow[:6],
                          " scrapeF=", round(self._carry_scrape, 1),
                          flush=True)
                    ray_from = [q[0], q[1], 0.72]
                    dvec = (np.asarray(ls, float)
                            if ls is not None else np.zeros(2))
                    dn = float(np.linalg.norm(dvec))
                    if dn > 1e-6:
                        dvec /= dn
                        hit = p.rayTest(
                            [q[0], q[1], 0.72],
                            [q[0] + 1.2 * dvec[0], q[1] + 1.2 * dvec[1],
                             0.72],
                            physicsClientId=self.env.pb_physics_client)
                        hit = [h for h in hit
                               if h[0] not in (self.env.object_id,
                                               self.env.robot_id)]
                        print("   ray +z0.72 along cmd hit:",
                              [(h[0], round(h[1], 3)) for h in hit],
                              flush=True)
                    # Drop the stale command anchor: a cmd_xy left
                    # leading/lagging the base by >1 cm made the target
                    # flip sign on alternating ticks, and the base drove
                    # a period-2 limit cycle (400 steps, 2.5 cm net) with
                    # zero world contacts -- not a jam, an oscillation.
                    self._carry_cmd_xy = base_xy.copy()
                    if self._carry_replans <= 1 or d_xy > 2.0:
                        # Blind servoing across the maze drives straight
                        # into the first wall the map does not show (shed
                        # at (1.9,-2.5) while 15 m from the goal). Keep
                        # replanning A* until actually near the stand cell.
                        self.carry_path = None
                    else:
                        print("Carry: stuck after replans; servoing.",
                              flush=True)
                        self._carry_servo = True
                else:
                    # Validate on the map A* actually planned on. Handing
                    # nav the payload-pruned map rejects every corridor
                    # path it just produced (invalid_grid_path_or_start),
                    # starving travel into stuck-replans and servo sheds.
                    # Wall scraping is caught live by the scrape guard.
                    step_xy = self.curved_nav.step(base_xy, self.carry_path,
                                                   self.carry_occupancy)
                    self._carry_last_step = step_xy
                    if self._carry_scrape > 20.0:
                        # Payload pressed on an obstacle: driving presses
                        # harder and the wall peels it out of the pads.
                        # Stop; the stuck timer will replan around it.
                        print(f"Carry: payload scraping world "
                              f"(F={self._carry_scrape:.0f} N); pausing.",
                              flush=True)
                        step_xy = np.zeros(2)
                    if np.linalg.norm(step_xy) > 0.01:
                        heading = float(np.arctan2(step_xy[1], step_xy[0]))
                    # No stop-and-turn gate here: travel strafes (yaw is
                    # frozen; steering only happens in the final stage), so
                    # a path tangent oblique to the frozen heading is driven
                    # sideways, not turned into. The old gate that zeroed XY
                    # whenever head_err > 0.35 deadlocked the base forever
                    # (2000 frozen ticks at 900 N grip, dist 17.9).
                    # Constant 3 cm lead anchored to the measured base (the
                    # stuck-mode/servo pattern that carried 1.5-4 m net at
                    # stable 440-500 N grip). The old 1 cm/step limiter kept
                    # the target ~1 cm from the base -- a distance the
                    # position servo out-runs -- so target and base passed
                    # each other every other tick and the base rocked in
                    # place: 400 steps, 2.5 cm net, qvel flipping +/-0.22,
                    # zero world contacts (the (-1.91,-0.01) freeze). A
                    # constant lead has no such equilibrium: the servo can
                    # only lag it.
                    cmd = np.clip(step_xy, -0.03, 0.03)
                    self._carry_cmd_xy = base_xy + cmd
                    # No lateral recenter here: a rigidly gripped payload
                    # holds a CONSTANT offset from the base (lat 0.062 held
                    # for 400 ticks at 800 N), so strafing to re-centre is
                    # physically futile -- it just biases the base off the
                    # A* corridor every tick, and the 0.78 m-ahead payload
                    # then grinds the wall and peels out of the pads.
                    cmd = np.clip(cmd, -0.05, 0.05)
                    action[0] = q[0] + float(cmd[0])
                    action[1] = q[1] + float(cmd[1])
                    status = self.curved_nav.status
                    if status != getattr(self, "_last_carry_status", None):
                        print("Carry navigation:", status)
                        self._last_carry_status = status
        if self._carry_stage == "final" or getattr(self, "_carry_servo", False):
            # Servo on the grasped object directly. The servo branch had no
            # scrape guard (travel did), so a payload pressed on an
            # obstacle kept being driven harder -- the (1.9,-2.5) shed was
            # a backward squirt out of exactly that press. Hold XY while
            # scraping; the grip survives static wall load, not grinding.
            if self._carry_scrape > 20.0:
                hold_n = getattr(self, "_carry_servo_hold", 0) + 1
                self._carry_servo_hold = hold_n
                print(f"Carry: servo payload scraping world "
                      f"(F={self._carry_scrape:.0f} N); holding {hold_n}.",
                      flush=True)
                if hold_n > 120:
                    # Servo mode has no stuck detector (it implies an empty
                    # path), so a permanent press would hold to timeout.
                    # Hand back to A*, which routes around the obstacle.
                    print("Carry: servo scrape hold expired; replanning.",
                          flush=True)
                    self._carry_servo = False
                    self._carry_servo_hold = 0
                    self.carry_path = None
            else:
                self._carry_servo_hold = 0
                action[0] = q[0] + float(np.clip(goal[0] - obj[0], -0.03, 0.03))
                action[1] = q[1] + float(np.clip(goal[1] - obj[1], -0.03, 0.03))

        err = (heading - q[2] + np.pi) % (2 * np.pi) - np.pi
        # No yaw while the payload is airborne: the apple rides 0.73 m
        # ahead and 0.21 m off-axis, and every mechanism tried so far that
        # rotated the base (even turn-in-place at full grip force ~800 N)
        # wrung the sphere out of the flat pads. The base is omnidirectional
        # -- travel strafes along the path tangent with yaw frozen; heading
        # only steers once the payload is down (final servo).
        if abs(err) > 0.08 and self._carry_stage == "final":
            action[2] = q[2] + float(np.clip(err * self.action_scale[2] * 5,
                                             -0.004, 0.004))

        # Travel height servo: the pads are only 12 mm tall and drop
        # telemetry showed the apple sliding DOWN off them during travel
        # (obj_z sagged, grip force decayed 383->235 N before the shed).
        # Creep the torso up whenever the payload falls below the grasp
        # height so the pads keep riding centred on the sphere.
        if self._carry_stage == "travel" and not getattr(self, "_carry_servo", False):
            z_ref = getattr(self, "_carry_z_ref", None)
            if z_ref is None:
                z_ref = self._carry_z_ref = float(obj[2])
            if obj[2] < z_ref - 0.005:
                if not hasattr(self, "_carry_torso_cmd"):
                    self._carry_torso_cmd = float(action[3])
                self._carry_torso_cmd = float(min(
                    self._carry_torso_cmd + 0.001,
                    self.action_upper_bound[3]))
                action[3] = self._carry_torso_cmd

        # Closed-loop object height: torso first, then shoulder_lift
        # (payload z tracks torso + arm, and the torso alone tops out below
        # the 0.9 m goal).
        if self._carry_stage == "final":
            if not hasattr(self, "_carry_torso_cmd"):
                self._carry_torso_cmd = float(q[3])
            if not hasattr(self, "_carry_lift_cmd"):
                self._carry_lift_cmd = float(q[7])
            corr = float(np.clip(goal[2] - obj[2], -0.01, 0.01))
            self._carry_torso_cmd = float(np.clip(
                self._carry_torso_cmd + corr, 0.0, self.action_upper_bound[3]))
            if self._carry_torso_cmd >= self.action_upper_bound[3] - 1e-6:
                self._carry_lift_cmd = float(np.clip(
                    self._carry_lift_cmd + corr,
                    self.action_lower_bound[7], self.action_upper_bound[7]))
            action[3] = self._carry_torso_cmd
            action[7] = self._carry_lift_cmd

        self.action = np.clip(action, self.action_lower_bound,
                              self.action_upper_bound)
        return self.action.copy()

    def _retry_after_grasp_failure(self):
        """Blacklist the parked cell and hand back to the base planner.

        Returns False when every approach cell is exhausted (terminal).
        """
        attempt = getattr(self, "grasp_attempt_goal", None)
        if (attempt is not None
                and not getattr(self, "_grasp_retry_exhausted", False)):
            self.navigate_grasp.mark_goal_failed(attempt)
        else:
            # The attempt was a direct-GRASP plan from wherever the base
            # happened to stand (typically the spawn cell). Blacklist that
            # cell so state 0 must navigate to a proper park pose; planning
            # from here again would only repeat the identical failure.
            obs = self.env._get_obs()
            base_xy = np.array([obs["qpos"][0] - 2.0, obs["qpos"][1]])
            self.navigate_grasp.mark_goal_failed(
                self.convert_to_world(
                    self.convert_to_grid(base_xy)))
        try:
            self.navigate_grasp.nearest_free_goal(
                np.asarray(self.env._get_obs()["object_pos"], float)[:2],
                self.inflated_occupancy, self.convert_to_grid,
                self.convert_to_world, self.cell_size)
        except RuntimeError:
            self._grasp_retry_exhausted = True
            return False
        self.navigate_grasp.grasp = None
        self.path = None
        self._path_goal = None
        self.waypoint_index = 0
        self.curved_nav.reset()
        self._travel_pose_complete = False
        # Release the closed-finger hold: re-approaching with pads dug into
        # the tabletop stalls the base against the table edge forever.
        self.action[13:15] = self.action_upper_bound[13:15]
        try:
            del self._moving_pose_target
        except AttributeError:
            pass
        self.state = 0
        return True

    def navigate(self, env, goal):
        obs = env._get_obs()
        qpos = obs["qpos"]

        # Keep moving toward the test pose until it has settled.
        if not getattr(self, "_travel_pose_complete", False):
            pose_action = self.moving_pose(obs)

            # The arm can press into the table before reaching the pose and
            # stall 0.03-0.07 rad from target under position control: with
            # no bound this branch held the base dead-still pushing into
            # the tabletop for 2800 ticks (r000). Bail to the backoff +
            # cell-blacklist path, same as a failed yaw.
            self._pose_wait = getattr(self, "_pose_wait", 0) + 1
            if self._pose_wait > 400:
                self._pose_wait = 0
                print("Travel pose unreachable here: backing off.")
                self._grasp_backoff = True
                self._backoff_ticks = 0
                return pose_action

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

        # (Re)plan when the latched goal changed: after a grasp failure the
        # navigator re-latches a new park cell, but a stale 'reached' path
        # used to freeze the base at the old stand spot forever.
        goal_cell = tuple(int(v) for v in goal)
        if self.path is None or getattr(self, "_path_goal", None) != goal_cell:
            self._path_goal = goal_cell
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
        else:
            # Empty path (typically 'start is blocked after inflation' --
            # the base drifted into a wall/obstacle inflation zone and A*
            # can see no route from here, so a held action froze the run
            # for 700 steps). Drive straight to the nearest free cell.
            free = np.argwhere(self.inflated_occupancy == 0)
            if len(free) == 0:
                raise RuntimeError("occupancy fully blocked")
            start_i = np.asarray(tuple(int(v) for v in start), dtype=int)
            tgt = free[np.argmin(np.linalg.norm(free - start_i, axis=1))]
            tgt_xy = np.asarray(self.convert_to_world(tuple(int(v) for v in tgt)),
                                float)
            step = tgt_xy - base_xy
            d = float(np.linalg.norm(step))
            if d > 1e-6:
                step *= min(d, 0.05) / d
            self.forward = step[0] / self.action_scale[0]
            self.turn = step[1] / self.action_scale[1]

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

            # Torso at full lift raised the shoulder ~0.38 m: pregrasp IK
            # residuals for table-top objects went from 30-70 mm misses
            # (torso=0) to sub-5 mm convergence (measured probe_ik_residual).
            target[3] = self.action_upper_bound[3]  # torso_lift

            # Illustrative test angles in radians.
            # These need validation on your robot; they are not a verified safe pose.
            target[3] = 0.1     # torso lift joint
            target[6] = 0.0     # shoulder_pan
            target[7] = -1.2    # shoulder_lift
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
        self.mark_obstacle_bounds(self.extra_occupancy, extra_bounds)

        self.occupancy = (
            self.wall_occupancy | self.extra_occupancy
        )

        # Carry map: whole-table footprint (no above-base-height exclusion).
        self.carry_extra_occupancy = np.zeros_like(occupancy)
        carry_bounds = self.get_extra_obstacle_bounds(
            cube_positions, full_footprint=True)
        self.mark_obstacle_bounds(self.carry_extra_occupancy, carry_bounds)

        print("Map created. Occupancy grid : ", occupancy)

    def _inflate_extra(self, extra, safety_margin):
        radius_cells = int(np.ceil(
            (self.robot_radius + safety_margin) / self.cell_size
        ))
        extra = extra.astype(bool)
        rows, cols = extra.shape
        padded = np.pad(extra, radius_cells, mode="constant")
        extra_blocked = extra.copy()
        for dr in range(2 * radius_cells + 1):
            for dc in range(2 * radius_cells + 1):
                extra_blocked |= padded[
                    dr:dr + rows,
                    dc:dc + cols,
                ]
        return extra_blocked

    def create_navigable_map(self):
        safety_margin = 0.05
        return (
            self.wall_blocked.astype(bool)
            | self._inflate_extra(self.extra_occupancy, safety_margin)
        ).astype(np.uint8)

    def create_carry_map(self):
        """A* map for loaded transport.

        Walls are inflated by the base disk only, not the projected link
        footprint: the extended arm makes the C-space projection seal every
        90-degree maze turn, while in travel the yaw follows the path
        tangent so the arm leads down the corridor and only the base disk
        has to fit. The table uses its whole footprint -- the payload
        sweeps over the tabletop, so the base must walk around it."""
        safety_margin = 0.05
        return (
            self._inflate_extra(self.wall_occupancy, safety_margin).astype(bool)
            | self._inflate_extra(self.carry_extra_occupancy, safety_margin)
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
    def get_extra_obstacle_bounds(self, cube_positions, *, full_footprint=False):
        """Read additional obstacle bounds without changing the simulation.

        full_footprint keeps the tall pieces (tabletop) so the map blocks the
        whole table -- used for carry planning, where the base must walk
        around the table rather than under it.
        """
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
                # out. Table legs start at z=0 and still count. Carry
                # planning (full_footprint) keeps them: the lifted payload
                # snags the tabletop edge and the legs catch the cart, so
                # the carry base must walk *around* the whole table.
                if lower[2] >= 0.35 and not full_footprint:
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