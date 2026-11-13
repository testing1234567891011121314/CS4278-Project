from collections import deque

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
        4. provide a free navigation target,
        5. return PREPARE_GRASP actions to turn and settle before planning.

    Reach bounds are horizontal parking heuristics, not an IK certificate.
    The occupancy grid must already be inflated for the robot footprint.
    Navigation and turning assume a collision-free transport arm posture;
    this helper does not check the full body's swept volume while turning.
    Callers must execute decision["action"] for PREPARE_GRASP and obtain a
    fresh observation before calling update again. Torso + arm planning is
    performed separately by GraspPlanner.plan(obs) after state == GRASP.
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
        *,
        base_x_offset=-2.0,
        preferred_reach=0.55,
        position_tolerance=0.06,
        yaw_tolerance=0.05,
        yaw_step=0.02,
        settle_steps=5,
        failed_goal_radius=0.12,
    ):
        self.env = env
        self.grasp_planner = grasp_planner

        self.min_reach = min_reach
        self.max_reach = max_reach
        self.min_z = min_z
        self.max_z = max_z
        self.max_lateral = max_lateral

        settings = np.array([min_reach, max_reach, min_z, max_z, max_lateral,
                             base_x_offset, preferred_reach, position_tolerance,
                             yaw_tolerance, yaw_step, failed_goal_radius], float)
        if (not np.isfinite(settings).all() or not 0 < min_reach < max_reach
                or min_z >= max_z or max_lateral <= 0
                or not min_reach <= preferred_reach <= max_reach
                or min(position_tolerance, yaw_tolerance, yaw_step,
                       failed_goal_radius) <= 0
                or not isinstance(settle_steps, int) or settle_steps < 1):
            raise ValueError("Invalid parking distances, tolerances or settling count.")
        self.base_x_offset = base_x_offset
        self.preferred_reach = preferred_reach
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.yaw_step = yaw_step
        self.settle_steps = settle_steps
        self.failed_goal_radius = failed_goal_radius
        self._object_pose = None
        self._prepare_target = None
        self._previous_q = None
        self._stable = self._prepare_ticks = 0
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
            qpos[0] + self.base_x_offset,
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

    @staticmethod
    def _angle_error(target, current):
        return float((target - current + np.pi) % (2 * np.pi) - np.pi)

    def classify_reachability(self, grasp_position_base):
        """Coarse parking check; only the 8-DOF planner checks actual reach."""
        x, y, z = grasp_position_base
        # Height is checked separately: the torso can change shoulder height.
        distance = np.hypot(x, y)
        if not self.min_z <= z <= self.max_z:
            return "HEIGHT_OUT_OF_RANGE"
        if distance > self.max_reach:
            return "MOVE_CLOSER"
        if distance < self.min_reach:
            return "MOVE_FARTHER"
        if x <= 0 or abs(y) > self.max_lateral:
            return "TURN_TO_OBJECT"
        return "WITHIN_REACH"

    def _reset_preparation(self):
        self._prepare_target = self._previous_q = None
        self._stable = self._prepare_ticks = 0

    def _failed(self, xy):
        return any(np.linalg.norm(np.asarray(xy) - g) <= self.failed_goal_radius
                   for g in self.failed_goals)

    def mark_goal_failed(self, goal_world):
        """Skip a neighbourhood after a failed attempt, not proven infeasibility.

        Pass the actual parked base WORLD xy, not qpos[:2] (which has an
        x offset in this environment). Budget exhaustion is not a proof.
        """
        xy = np.asarray(goal_world, float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError("goal_world must be finite world xy.")
        self.failed_goals.add(tuple(xy))
        self.navigation_goal = None
        self._reset_preparation()

    @staticmethod
    def _free_cell(xy, occupancy, convert_to_grid):
        cell = tuple(np.asarray(convert_to_grid(xy), dtype=int))
        return (len(cell) == 2 and 0 <= cell[0] < occupancy.shape[0]
                and 0 <= cell[1] < occupancy.shape[1]
                and occupancy[cell] == 0)

    def nearest_free_goal(self, object_xy, occupancy, convert_to_grid,
                          convert_to_world, cell_size=0.1, robot_xy=None):
        """Choose a connected free parking cell near the preferred standoff.

        World coordinates determine distances; cell_size is retained only for
        call compatibility. Connectivity is a grid check, not a full-body path
        check. Diagonals cannot cut across blocked corners.
        """
        occupancy = np.asarray(occupancy)
        if occupancy.ndim != 2:
            raise ValueError("occupancy must be a 2D inflated grid.")
        free = occupancy == 0
        if robot_xy is not None:
            if not self._free_cell(robot_xy, occupancy, convert_to_grid):
                raise RuntimeError("Current base cell is blocked/outside the inflated grid.")
            start = tuple(np.asarray(convert_to_grid(robot_xy), dtype=int))
            connected = np.zeros_like(free)
            connected[start] = True
            queue = deque([start])
            while queue:
                a, b = queue.popleft()
                for da, db in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                    i, j = a + da, b + db
                    if not (0 <= i < free.shape[0] and 0 <= j < free.shape[1]):
                        continue
                    if not free[i,j] or connected[i,j]:
                        continue
                    if da and db and not (free[a+da,b] and free[a,b+db]):
                        continue
                    connected[i,j] = True
                    queue.append((i,j))
            free &= connected
        cells = np.argwhere(free)
        if not len(cells):
            raise RuntimeError("No connected free parking cells.")
        world = np.asarray([convert_to_world(tuple(c)) for c in cells], float)
        distances = np.linalg.norm(world - np.asarray(object_xy), axis=1)
        allowed = ((distances >= self.min_reach) & (distances <= self.max_reach)
                   & np.array([not self._failed(xy) for xy in world]))
        if not np.any(allowed):
            raise RuntimeError("No untried connected parking cell in the configured "
                               "distance range; this does not prove the grasp impossible.")
        world, distances = world[allowed], distances[allowed]
        travel = (np.linalg.norm(world - robot_xy, axis=1)
                  if robot_xy is not None else np.zeros(len(world)))
        # Prefer a useful standoff, with a smaller preference for nearby cells.
        score = np.abs(distances - self.preferred_reach) + 0.15 * travel
        return tuple(world[np.argmin(score)])

    def _prepare(self, obs, target_yaw):
        """Absolute action targets: hold x/y, torso/head/arm/fingers; turn yaw."""
        q = np.asarray(obs["qpos"], float)
        if self._prepare_target is None:
            self._prepare_target = q.copy()
        self._prepare_ticks += 1
        if self._prepare_ticks > 600:
            raise RuntimeError("Base alignment/settling timed out; inspect tracking and collisions.")
        error = self._angle_error(target_yaw, q[2])
        action = self._prepare_target.copy()
        action[2] = q[2] + np.clip(error, -self.yaw_step, self.yaw_step)
        # Choose the nearest equivalent yaw that the environment can command.
        if hasattr(self.env, "joint_lower") and hasattr(self.env, "joint_upper"):
            lo, hi = self.env.joint_lower[2], self.env.joint_upper[2]
            if lo <= hi:
                if np.isfinite(lo):
                    action[2] += 2*np.pi * max(0, np.ceil((lo-action[2])/(2*np.pi)))
                if np.isfinite(hi):
                    action[2] -= 2*np.pi * max(0, np.ceil((action[2]-hi)/(2*np.pi)))
                if not lo <= action[2] <= hi:
                    raise RuntimeError("Required base yaw cannot satisfy the joint limits.")
        settled = False
        if self._previous_q is not None:
            delta = q - self._previous_q
            delta[2] = self._angle_error(q[2], self._previous_q[2])
            settled = np.max(np.abs(delta)) < 0.0005
        if "qvel" in obs:
            velocity = np.asarray(obs["qvel"], float)
            settled = (settled and velocity.shape == q.shape
                       and np.isfinite(velocity).all()
                       and np.max(np.abs(velocity)) < 0.01)
        self._previous_q = q.copy()
        held_error = q - self._prepare_target
        held_error[2] = 0  # yaw has its own target and tolerance
        ready = (abs(error) <= self.yaw_tolerance and settled
                 and np.max(np.abs(held_error)) < 0.01)
        self._stable = self._stable + 1 if ready else 0
        return action, self._stable >= self.settle_steps

    def update(self, obs, occupancy, convert_to_grid, convert_to_world, cell_size=0.1):
        """Return navigation, PREPARE_GRASP (with action), or GRASP.

        PREPARE_GRASP must be executed by the caller; ignoring it leaves the
        robot waiting forever. No torso height or arm ready pose is guessed.
        """
        q = np.asarray(obs["qpos"], float)
        if q.ndim != 1 or len(q) < 3 or not np.isfinite(q).all():
            raise ValueError("Invalid measured qpos.")
        occupancy = np.asarray(occupancy)
        cloud = np.asarray(obs["object_pc"], float)
        if cloud.ndim != 2 or cloud.shape[1] != 3 or not len(cloud) or not np.isfinite(cloud).all():
            raise ValueError("object_pc must contain finite world-frame points.")
        position = np.asarray(obs.get("object_pos", cloud.mean(axis=0)), float)
        quat = np.asarray(obs.get("object_xyzw", [0,0,0,1]), float)
        if (position.shape != (3,) or quat.shape != (4,)
                or not np.isfinite(position).all() or not np.isfinite(quat).all()
                or np.linalg.norm(quat) < 1e-12):
            raise ValueError("Invalid object pose.")
        quat = quat / np.linalg.norm(quat)
        if self._object_pose is not None:
            old_pos, old_quat = self._object_pose
            angle = 2*np.arccos(np.clip(abs(np.dot(quat, old_quat)), 0, 1))
            if np.linalg.norm(position-old_pos) > 0.01 or angle > 0.10:
                self.grasp = self.navigation_goal = None
                self.failed_goals.clear()
                self._reset_preparation()
                self._object_pose = None
        if self._object_pose is None:
            self._object_pose = (position.copy(), quat.copy())
        if self.grasp is None:
            self.select_grasp(obs)
        base, yaw = self.get_base_pose(obs)
        grasp_pos, grasp_quat = self.relative_grasp_pose(obs)
        reach = self.classify_reachability(grasp_pos)
        if reach == "HEIGHT_OUT_OF_RANGE":
            raise RuntimeError("Grasp height is outside the configured coarse height range; "
                               "moving the base horizontally will not change it.")
        target = self.grasp["position_world"][:2]
        distance = np.linalg.norm(target-base[:2])
        current_ok = (self.min_reach <= distance <= self.max_reach
                      and not self._failed(base[:2])
                      and self._free_cell(base[:2], occupancy, convert_to_grid))
        result = dict(grasp_position_base=grasp_pos, grasp_orientation_base=grasp_quat,
                      grasp_position_world=self.grasp["position_world"],
                      grasp_orientation_world=self.grasp["orientation_world"])
        if self.navigation_goal is not None:
            goal = np.asarray(self.navigation_goal)
            if (self._failed(goal) or not self._free_cell(goal, occupancy, convert_to_grid)):
                self.navigation_goal = None
        if not current_ok or self.navigation_goal is not None:
            if self.navigation_goal is None:
                self.navigation_goal = self.nearest_free_goal(
                    target, occupancy, convert_to_grid, convert_to_world,
                    cell_size, robot_xy=base[:2])
            arrived = np.linalg.norm(base[:2]-self.navigation_goal) <= self.position_tolerance
            if not arrived or not current_ok:
                self._reset_preparation()
                return dict(result, state="NAVIGATE_TO_GRASP",
                            navigation_goal=self.navigation_goal,
                            navigation_yaw=float(np.arctan2(
                                *(target-np.asarray(self.navigation_goal))[::-1])))
        target_yaw = float(np.arctan2(target[1]-base[1], target[0]-base[0]))
        action, ready = self._prepare(obs, target_yaw)
        return dict(result, state="GRASP" if ready else "PREPARE_GRASP",
                    navigation_goal=None, navigation_yaw=target_yaw,
                    action=None if ready else action)
