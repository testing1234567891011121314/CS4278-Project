"""Pick-Nav-Reach solver: one state machine, one action per tick.

Drop-in for the course's KeyBoardController (R1):

    solver = Solver(env)
    while True:
        obs, info = env.step(solver.get_action())

See docs/plan.md for the segment each state belongs to. NAVIGATE and REACH
(segment 3) are not written yet: after the back-off the solver holds its pose.
"""
from __future__ import annotations

import dataclasses
import traceback
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from custom_libraries.collision import CollisionChecker
from custom_libraries.common import Deadline, Status
from custom_libraries.grasping import arm_planner, ik
from custom_libraries.grasping.arm_planner import GraspPlan
from custom_libraries.grasping.candidates import GraspCandidate, antipodal_candidates, rake_candidates
from custom_libraries.grasping.executor import GraspExecutor, PathStepper, grasp_error, is_held
from custom_libraries.navigation.base_follower import BaseFollower
from custom_libraries.navigation.grid import OccupancyGrid
from custom_libraries.navigation.parking import (ParkingPose, ParkingSettings, choose_parking, make_follower,
                                                  travel_order)
from custom_libraries.planning_sim import PlanningSim, rotation
from custom_libraries.robot import RobotModel
from custom_libraries.world import ObjectState, RestDetector, World


class State(Enum):
    SETTLE = "settle"                # segment 1: wait for the object to come to rest
    GO_TO_OBJECT = "go_to_object"    # segment 1: choose a parking pose and drive to it
    GRASP = "grasp"                  # segment 2: plan, close, verify, lift, hold
    DRAG = "drag"                    # segment 2: drag a gripped object to the edge of what it lies on
    BACK_OFF = "back_off"            # segment 3: back away from the table with the object
    NAVIGATE = "navigate"            # segment 3: carry the object through the maze
    REACH = "reach"                  # segment 3: bring the object to the goal
    RETRY = "retry"                  # segment 4: decide what to do after a failure
    DONE = "done"                    # delivered; hold the pose
    STOPPED = "stopped"              # out of ticks; hold the pose


TERMINAL = frozenset({State.DONE, State.STOPPED})


@dataclass(frozen=True)
class Config:
    """Every setting of the solver. Never mutated (R25)."""

    # From the rubric and the user's cap (docs/requirements.md).
    episode_ticks: int = 3000        # R16
    lift_deadline_tick: int = 600    # R9
    lift_height: float = 0.10        # R10, metres
    hold_ticks: int = 50             # R11
    goal_tolerance: float = 0.10     # R15, metres

    # Tick budget of each state (R19). The first three add up to the lift deadline.
    settle_ticks: int = 150
    go_to_object_ticks: int = 190
    grasp_ticks: int = 260
    drag_ticks: int = 260
    back_off_ticks: int = 260
    navigate_ticks: int = 2000
    reach_ticks: int = 200
    settle_wait_ticks: int = 120     # an object still moving after this is approached as it is
    min_retry_ticks: int = 300       # no new attempt is started with fewer ticks left
    max_attempts: int = 8            # nor after this many attempts

    # CPU-time budgets of the planning calls (R26), seconds.
    ready_seconds: float = 3.0
    parking_seconds: float = 20.0
    planning_seconds: float = 20.0

    # Geometry of the drive.
    grid_resolution: float = 0.10    # metres per cell of the floor map
    wall_margin: float = 0.06        # the base keeps this far from maze walls, beyond its own radius
    table_keepout: float = 0.12      # the base centre keeps this far from a table's outline on its way
    ready_reach: float = 0.45        # the ready pose holds the hand this far in front of the base centre
    ready_height: float = 0.35       # ... and this far above the highest surface (table or floor)
    back_off: tuple = (0.6, 0.45, 0.3)   # distances tried for the back-off, metres
    carry_step: float = 0.02         # base speed with the object in the hand, metres per tick
    carry_ramp: float = 0.002        # change of that step per tick
    min_carry_step: float = 0.003    # slowest the base carries, however heavy the object
    carry_safety: float = 0.5        # share of the grip's friction the base's jolts may use
    base_gain: float = 0.3           # the env's position gain on the base joints (set in its step())
    finger_stiffness: float = 6000.0  # newtons per metre the finger target lies inside the object (measured:
                                     # 12.5 N at 1.9 mm, 180 N at 30 mm)
    squeeze: float = 0.030           # how far inside the object the finger targets are put, metres
    grip_safety: float = 2.0         # the grip's friction must hold this many times the object's weight
    moment_safety: float = 2.0       # the pads must resist this many times the weight's turning moment, or
                                     # the object is first dragged to the edge of what it lies on
    overhang: float = 0.6            # share of the object's half-length left hanging over the edge by a drag
    drag_step: float = 0.006         # base speed while raking, metres per tick: at 20 mm per tick the plate
                                     # kept sliding when the hand stopped and fell off the table
    drag_slack: float = 0.08         # a dragged object may trail the base by this much, metres
    rake_clear: float = 0.35         # an object raked out from under something ends this far beyond its edge
    max_found: int = 1600            # grasp candidates taken from the cloud
    max_candidates: int = 60         # of those the hand fits, the best this many go on

    seed: Optional[int] = None       # R24; None draws fresh entropy
    strict: bool = False             # tests: let exceptions out of get_action

    def budget(self, state: State) -> int:
        """Ticks the solver may spend in `state` before it counts as a failure."""
        return {
            State.SETTLE: self.settle_ticks,
            State.GO_TO_OBJECT: self.go_to_object_ticks,
            State.GRASP: self.grasp_ticks,
            State.DRAG: self.drag_ticks,
            State.BACK_OFF: self.back_off_ticks,
            State.NAVIGATE: self.navigate_ticks,
            State.REACH: self.reach_ticks,
        }.get(state, self.episode_ticks)


@dataclass
class Scene:
    """What the solver knows about the robot and its surroundings for the whole trial."""

    robot: RobotModel
    sim: PlanningSim
    collision: CollisionChecker
    world: World
    grid: OccupancyGrid


@dataclass
class Attempt:
    """Everything one pick attempt knows. A retry gets a new one (R21)."""

    index: int
    started_tick: int
    pick_mode: Optional[str] = None                 # "table" or "floor"
    rest: RestDetector = field(default_factory=RestDetector)
    unwind: Optional[np.ndarray] = None             # (K, 15) the way back out of the last attempt's grasp
    arm: Optional[PathStepper] = None               # the move to the ready pose
    object_at_plan: Optional[np.ndarray] = None     # where the object was when the drive was planned
    candidates: list[GraspCandidate] = field(default_factory=list)
    tiers: list[int] = field(default_factory=list)  # rank of each candidate that comes before its score
    parking: Optional[ParkingPose] = None
    drive: Optional[BaseFollower] = None
    grasp_plan: Optional[GraspPlan] = None
    executor: Optional[GraspExecutor] = None
    retreat: Optional[BaseFollower] = None
    drag_from: Optional[tuple] = None               # where the base and the object were when a drag began
    lost: int = 0                                   # ticks in a row without the hold, after the grasp
    search: Optional[dict] = None                   # what the candidate and parking searches found
    plan_stats: Optional[dict] = None
    lifted_tick: Optional[int] = None
    grasp_error: Optional[tuple] = None             # metres, radians (R12)
    failure: Optional[str] = None


@dataclass
class Report:
    """What the tools may read about a trial, besides `solver.state` (R27)."""

    settings: dict
    transitions: list[tuple[int, str]] = field(default_factory=list)   # (tick, state entered)
    attempts: list[dict] = field(default_factory=list)                 # one summary per finished attempt
    errors: list[tuple[int, str, str]] = field(default_factory=list)   # (tick, state, traceback)
    delivered_tick: Optional[int] = None


class Solver:
    def __init__(self, env, config: Optional[Config] = None, start: State = State.SETTLE):
        self.env = env
        self.config = config or Config()
        self.rng = np.random.default_rng(self.config.seed)
        self.tick = 0
        self.state = start
        self.attempt = Attempt(index=0, started_tick=0)
        self.report = Report(settings=dataclasses.asdict(self.config))
        self.report.transitions.append((0, start.value))
        self.scene: Optional[Scene] = None
        self._state_entered = 0
        self._hold = self._measured_pose()       # last commanded action, latched (R23)
        self._tried_parking: list[np.ndarray] = []       # (x, y) of parking poses that led nowhere
        self._tried_grasps: list[np.ndarray] = []        # centres of failed grasps, in the object's frame
        self._dragged = False                            # the object has been dragged to an edge in this trial
        self._handlers = {
            State.SETTLE: self._settle,
            State.GO_TO_OBJECT: self._go_to_object,
            State.GRASP: self._grasp,
            State.DRAG: self._drag,
            State.BACK_OFF: self._back_off,
            State.NAVIGATE: self._navigate,
            State.REACH: self._reach,
            State.RETRY: self._retry,
        }

    def close(self) -> None:
        """Release the planning simulator."""
        if self.scene is not None:
            self.scene.sim.close()
            self.scene = None

    # ------------------------------------------------------------------ spine

    def get_action(self, env=None) -> np.ndarray:
        """One (15,) array of absolute joint targets. Never raises (R18)."""
        if env is not None:
            self.env = env
        try:
            action = self._step(self.env._get_obs())
        except Exception:
            if self.config.strict:
                raise
            self.report.errors.append((self.tick, self.state.value, traceback.format_exc()))
            # A failure inside RETRY itself cannot be retried.
            self._fail("exception", to=State.STOPPED if self.state is State.RETRY else State.RETRY)
            action = self._hold
        self.tick += 1
        return action.copy()

    def _step(self, obs) -> np.ndarray:
        if self.state in TERMINAL:
            return self._hold
        if self.tick >= self.config.episode_ticks:
            self._fail("episode budget spent", to=State.STOPPED)
            return self._hold
        if self.tick - self._state_entered >= self.config.budget(self.state):
            self._fail(f"{self.state.value} budget spent")
            return self._hold
        action, next_state = self._handlers[self.state](obs)
        self._hold = np.asarray(action, dtype=float).copy()
        if next_state is not None:
            self._enter(next_state)
        return self._hold

    def _enter(self, state: State) -> None:
        self.state = state
        self._state_entered = self.tick
        self.report.transitions.append((self.tick, state.value))

    def _fail(self, reason: str, to: State = State.RETRY) -> None:
        if self.state in TERMINAL:
            return
        self.attempt.failure = reason
        self._enter(to)

    def _failed(self, reason: str) -> tuple[np.ndarray, Optional[State]]:
        """A handler's way out on failure: hold the pose and go to RETRY."""
        self.attempt.failure = reason
        return self._hold, State.RETRY

    def _new_attempt(self) -> None:
        """Close the current attempt in the report and start a fresh one (R21)."""
        a = self.attempt
        self.report.attempts.append(dict(index=a.index, started_tick=a.started_tick, pick_mode=a.pick_mode,
                                         lifted_tick=a.lifted_tick, failure=a.failure, search=a.search,
                                         plan_stats=a.plan_stats, grasp_error=a.grasp_error))
        self.attempt = Attempt(index=a.index + 1, started_tick=self.tick)

    def _measured_pose(self) -> np.ndarray:
        try:
            return np.asarray(self.env._get_obs()["qpos"], dtype=float).copy()
        except Exception:
            if self.config.strict:
                raise
            return np.zeros(int(getattr(self.env, "action_size", 15)))

    def _ticks_in_state(self) -> int:
        return self.tick - self._state_entered

    # ------------------------------------------------------------------ scene

    def _scene(self, obs) -> Scene:
        """Read the robot and the surroundings once per trial and build the
        planning simulator. Nothing is written to the env's simulator (R30)."""
        if self.scene is None:
            sim = PlanningSim(self.env)
            world = World.from_env(self.env, obs)
            grid = OccupancyGrid.from_world(world, self.config.grid_resolution,
                                            sim.base_radius + self.config.wall_margin,
                                            obstacle_radius=self.config.table_keepout)
            self.scene = Scene(robot=sim.robot, sim=sim, collision=CollisionChecker(sim), world=world, grid=grid)
        return self.scene

    def _ready_pose(self, scene: Scene, q: np.ndarray) -> Optional[np.ndarray]:
        """Joint values with the open hand pointing down in front of the base,
        above the highest surface nearby: the pose the base drives in."""
        robot, config = scene.robot, self.config
        x, y, yaw = robot.base_pose(q)
        forward, left = np.array([np.cos(yaw), np.sin(yaw), 0.0]), np.array([-np.sin(yaw), np.cos(yaw), 0.0])
        surface = max([scene.world.floor] + [float(box.upper[2]) for box in scene.world.obstacles])
        position = np.array([x, y, 0.0]) + config.ready_reach * forward + [0.0, 0.0, surface + config.ready_height]
        down = np.array([0.0, 0.0, -1.0])
        turn = np.column_stack([down, left, np.cross(down, left)])
        opened = q.copy()
        opened[list(robot.fingers)] = robot.upper[list(robot.fingers)]
        scene.collision.release()
        scene.collision.focus([x, y])
        deadline = Deadline(config.ready_seconds)
        for _ in range(4):
            ready = ik.solve(scene.sim, position, turn, opened, self.rng, deadline)
            if ready is None:
                return None
            if scene.collision.is_free(ready, 0.02):
                return ready
        return None

    # ----------------------------------------------------------------- states
    # Each handler takes the observation and returns (action, next state or
    # None to stay). A failure is `return self._failed(reason)`.

    def _settle(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 1. Move the arm to the ready pose while the object comes to
        rest. Exit: GO_TO_OBJECT, also when the object is still moving after
        `settle_wait_ticks` (it is then approached as it is)."""
        scene, attempt = self._scene(obs), self.attempt
        if attempt.arm is None:
            start = self._hold.copy() if attempt.unwind is None else attempt.unwind[-1].copy()
            ready = self._ready_pose(scene, start)
            path = None
            if ready is not None:
                scene.sim.set_object(obs["object_pos"], obs["object_xyzw"])
                path = arm_planner.plan_transit(start, ready, scene.sim, scene.collision, self.rng,
                                                Deadline(self.config.ready_seconds),
                                                avoid_object=attempt.unwind is None)
            if path is None:                         # stay as we are, with the hand open
                path = np.array([start, start])
                path[1, list(scene.robot.fingers)] = scene.robot.upper[list(scene.robot.fingers)]
            if attempt.unwind is not None:
                path = np.vstack([attempt.unwind, path[1:]])
            limit = np.full(len(start), 0.06)
            limit[scene.robot.torso] = 0.012
            limit[list(scene.robot.fingers)] = 0.01
            attempt.arm = PathStepper(path, limit)
        action = attempt.arm.step()
        at_rest = attempt.rest.update(ObjectState.from_obs(obs))
        waited = self._ticks_in_state() >= self.config.settle_wait_ticks
        return action, (State.GO_TO_OBJECT if attempt.arm.finished and (at_rest or waited) else None)

    def _go_to_object(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 1. Generate grasp candidates, choose a parking pose from
        which one is within reach, and drive there facing the object.
        Exit: GRASP. Failures: no candidates, no parking pose, base blocked,
        object moved."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        obj = ObjectState.from_obs(obs)
        if attempt.drive is None:
            # The surface under the object's top: an object wedged into a
            # table still has the table, not the floor, as its support.
            support = scene.world.support_under(np.array([*obj.centre[:2], obj.cloud[:, 2].max()]))
            attempt.pick_mode = "floor" if support <= scene.world.floor + 1e-6 else "table"
            top = float(obj.cloud[:, 2].max())
            found = antipodal_candidates(obj, scene.sim.gripper,
                                         lambda xy: scene.world.support_under(np.array([xy[0], xy[1], top])),
                                         limit=config.max_found)
            strong = [c for c in found if self._can_carry(c)]
            candidates = arm_planner.hands_that_fit(strong, obj, scene.sim, scene.collision)[:config.max_candidates]
            # Tiers, tried in this order whatever the scores: grasps that can
            # resist the weight's turning moment before those that cannot;
            # where the object hangs over an edge, grasps on the overhanging
            # part (the fingers have room below it) before the rest; grasps
            # with at least half the pad on the object before pinches with
            # the pad tips, which a few millimetres of slip undo; grasps not
            # tried before in this trial before those that failed.
            under = [scene.world.support_under(np.array([c.centre[0], c.centre[1], top])) for c in candidates]
            frame = rotation(obj.orientation)
            tiers = []
            for c, height in zip(candidates, under):
                local = frame.T @ (c.centre - obj.position)
                tried = any(np.linalg.norm(local - t) < 0.015 for t in self._tried_grasps)
                shallow = c.offset > (scene.sim.gripper.pad_tip + scene.sim.gripper.pad_back) / 2.0 + \
                    (scene.sim.gripper.pad_tip - scene.sim.gripper.pad_back) / 4.0
                tiers.append(8 * tried + 4 * self._cannot_lift(c, obj) +
                             2 * (height > min(under) + 0.05) + 1 * shallow)
            # If nothing can lift the object as it lies on a table, rake it
            # to the table's edge first: there the hand can reach round it.
            rake = attempt.pick_mode == "table" and not self._dragged and all(t & 4 for t in tiers)
            toward = scene.robot.base_pose(self._hold)[:2] - obj.centre[:2]
            if rake:
                candidates = rake_candidates(obj, scene.sim.gripper, support, toward)
                tiers = [0] * len(candidates)
            if not candidates:
                attempt.search = dict(found=len(found), candidates=0)
                return self._failed("no grasp candidates")
            attempt.candidates, attempt.tiers = candidates, tiers
            attempt.object_at_plan = obj.position.copy()
            attempt.parking, stats = choose_parking(attempt.candidates, obj, scene.grid, scene.sim,
                                                    scene.collision, self._hold, self.rng,
                                                    Deadline(config.parking_seconds),
                                                    exclude=tuple(self._tried_parking),
                                                    facing=toward if rake else None, tiers=tiers)
            if attempt.parking is None and not rake and not self._dragged and attempt.pick_mode == "floor":
                # Out of reach where it lies, under something: rake it out
                # the nearest way, to where the base can stand round it.
                way_out = self._way_out_from_under(obj)
                if way_out is not None:
                    rake, toward = True, way_out
                    candidates = rake_candidates(obj, scene.sim.gripper, scene.world.floor, toward)
                    attempt.candidates, attempt.tiers = candidates, [0] * len(candidates)
                    attempt.parking, stats = choose_parking(candidates, obj, scene.grid, scene.sim,
                                                            scene.collision, self._hold, self.rng,
                                                            Deadline(config.parking_seconds), facing=toward)
            attempt.search = dict(found=len(found), candidates=len(candidates),
                                  kinds=sorted({c.kind for c in candidates}),
                                  object=np.round(obj.position, 3).tolist(), parking=stats)
            if attempt.parking is None:
                return self._failed("no parking pose")
            yaw = scene.robot.base_pose(self._hold)[2]
            attempt.drive = make_follower(attempt.parking.path, yaw, attempt.parking.yaw, ParkingSettings())
        if np.linalg.norm(obj.position - attempt.object_at_plan) > 0.02:
            return self._failed("object moved during the drive")
        target, status = attempt.drive.step(scene.robot.base_pose(obs["qpos"]))
        action = scene.robot.with_base(self._hold, target)
        if status is Status.FAILED:
            self._tried_parking.append(attempt.parking.pose[:2])
            return self._failed(attempt.drive.failure)
        return action, (State.GRASP if status is Status.DONE else None)

    def _grasp(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 2. Plan the arm motion for the best candidate that plans,
        execute it, close, verify the hold, lift, hold.
        Exit: BACK_OFF. Failures: no plan, object moved, hold not verified,
        object lost."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        if attempt.executor is None:
            obj = ObjectState.from_obs(obs)
            scene.sim.sync(obs)
            # Candidates known to be within reach first, then the rest; in
            # each group by tier, then those that close along the line to the object first.
            toward = obj.centre[:2] - scene.robot.base_pose(self._hold)[:2]
            ranked = travel_order(attempt.candidates, toward, ParkingSettings().travel_weight, attempt.tiers)
            known = set(attempt.parking.reachable) if attempt.parking else set()
            order = [k for k in ranked if k in known] + [k for k in ranked if k not in known]
            plan, stats = arm_planner.plan_grasp([attempt.candidates[k] for k in order], self._hold, obj,
                                                 scene.sim, scene.collision, self.rng,
                                                 Deadline(config.planning_seconds),
                                                 drag_for=lambda c: c.kind == "rake")
            attempt.plan_stats = stats
            if plan is None:
                if attempt.parking is not None:
                    self._tried_parking.append(attempt.parking.pose[:2])
                return self._failed("no grasp plan")
            attempt.grasp_plan = plan
            c = plan.candidate
            stats["grasp"] = dict(kind=c.kind, width_mm=round(1000 * c.width, 1), opening_mm=round(1000 * c.opening, 1),
                                  offset_mm=round(1000 * c.offset, 1), approach=np.round(c.approach_axis, 2).tolist(),
                                  closing=np.round(c.closing_axis, 2).tolist(), lift_m=round(plan.lift_height, 3))
            attempt.executor = GraspExecutor(plan, self.env, scene.sim, config.lift_height, config.hold_ticks)
        action, status = attempt.executor.step(obs)
        if attempt.executor.phase == "hold" and attempt.lifted_tick is None:
            attempt.lifted_tick = self.tick
        if status is Status.FAILED:
            plan = attempt.grasp_plan
            attempt.plan_stats["failed_in"] = attempt.executor.phase
            self._tried_grasps.append(plan.grasp_in_object[0] + plan.grasp_in_object[1][:, 0] * plan.candidate.offset)
            return self._failed(attempt.executor.failure)
        if status is Status.DONE:
            attempt.grasp_error = grasp_error(self.env, scene.robot, attempt.grasp_plan, obs)
            return action, (State.DRAG if attempt.grasp_plan.drag else State.BACK_OFF)
        return action, None

    def _drag(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 2. With the open hand behind the object, back the base
        away so the fingers pull the object to the edge of what it lies on,
        where it hangs over; then take the hand away. From there the hand can
        reach round the object properly. Exit: RETRY, which starts the pick
        again."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        robot = scene.robot
        if attempt.retreat is None:
            self._dragged = True
            obj = ObjectState.from_obs(obs)
            base = robot.base_pose(self._hold)
            outward = base[:2] - obj.centre[:2]
            outward = outward / max(np.linalg.norm(outward), 1e-9)
            distance = self._drag_distance(obj, outward)
            step = config.drag_step
            attempt.plan_stats["drag_m"] = round(distance, 3)
            attempt.drag_from = (base[:2].copy(), obj.position[:2].copy())
            if distance > 0.02:
                path = np.array([base[:2], base[:2] + distance * outward])
                follower = BaseFollower(path, base[2], None, step, 0.03, step / 10.0)
                scene.collision.focus(base[:2])
                scene.collision.release()
                # The fingertips ride just above the support, so only a small gap is asked for.
                if all(scene.collision.is_free(robot.with_base(self._hold, pose), 0.002)
                       for pose in follower.poses()):
                    attempt.retreat = follower
            if attempt.retreat is None:
                return self._let_go("nowhere to drag the object to")
        # The object should come along: if the base has gone well ahead of it, the fingers have lost it.
        base_moved = np.linalg.norm(robot.base_pose(obs["qpos"])[:2] - attempt.drag_from[0])
        object_moved = np.linalg.norm(np.asarray(obs["object_pos"], dtype=float)[:2] - attempt.drag_from[1])
        if base_moved - object_moved > config.drag_slack:
            return self._let_go("object left behind during the drag")
        target, status = attempt.retreat.step(robot.base_pose(obs["qpos"]))
        if status is Status.FAILED:
            return self._let_go(attempt.retreat.failure)
        if status is Status.DONE:
            return self._let_go("dragged to the edge", robot.with_base(self._hold, target))
        return robot.with_base(self._hold, target), None

    def _let_go(self, reason: str, action: Optional[np.ndarray] = None) -> tuple[np.ndarray, Optional[State]]:
        """Open the hand where it is and start the pick again."""
        action = self._hold.copy() if action is None else action
        fingers = list(self.scene.robot.fingers)
        action[fingers] = self.scene.robot.upper[fingers]
        self.attempt.failure = reason
        return action, State.RETRY

    def _drag_distance(self, obj: ObjectState, outward: np.ndarray) -> float:
        """How far the object has to slide along `outward` for the set share
        of its half-length to hang over the edge of what it lies on."""
        centre = obj.centre[:2]
        top = float(obj.cloud[:, 2].max())
        for box in self.scene.world.obstacles:
            inside = np.all(box.lower[:2] <= centre) and np.all(centre <= box.upper[:2]) and box.upper[2] <= top + 0.02
            if not inside:
                continue
            to_edge = min(((box.upper[k] if outward[k] > 0 else box.lower[k]) - centre[k]) / outward[k]
                          for k in range(2) if abs(outward[k]) > 1e-9)
            half = float(((obj.cloud[:, :2] - centre) @ outward).max())
            return float(to_edge - half + self.config.overhang * half)
        # On the floor under something: out from under it, and a little farther.
        for box in self.scene.world.obstacles:
            if np.all(box.lower[:2] <= centre) and np.all(centre <= box.upper[:2]):
                to_edge = min(((box.upper[k] if outward[k] > 0 else box.lower[k]) - centre[k]) / outward[k]
                              for k in range(2) if abs(outward[k]) > 1e-9)
                return float(to_edge + self.config.rake_clear)
        return 0.0

    def _way_out_from_under(self, obj: ObjectState) -> Optional[np.ndarray]:
        """Horizontal direction of the shortest way out from under the fixed
        body the object lies beneath, or None if nothing is over it."""
        centre = obj.centre[:2]
        for box in self.scene.world.obstacles:
            if np.all(box.lower[:2] <= centre) and np.all(centre <= box.upper[:2]):
                gaps = [(centre[0] - box.lower[0], [-1.0, 0.0]), (box.upper[0] - centre[0], [1.0, 0.0]),
                        (centre[1] - box.lower[1], [0.0, -1.0]), (box.upper[1] - centre[1], [0.0, 1.0])]
                return np.array(min(gaps, key=lambda g: g[0])[1])
        return None

    def _back_off(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Back the base straight away from where the object stood,
        slowly, with the arm still. Exit: NAVIGATE. Failures: object lost,
        base blocked."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        robot = scene.robot
        if attempt.retreat is None:
            base = robot.base_pose(self._hold)
            origin = attempt.object_at_plan if attempt.object_at_plan is not None else obs["object_pos"]
            outward = base[:2] - np.asarray(origin, dtype=float)[:2]
            outward = outward / max(np.linalg.norm(outward), 1e-9)
            scene.collision.focus(base[:2])
            scene.collision.hold(self._hold, obs["object_pos"], obs["object_xyzw"])
            step = self._carry_step(outward)
            attempt.plan_stats["carry_step_mm"] = round(1000 * step, 1)
            for distance in config.back_off:
                path = np.array([base[:2], base[:2] + distance * outward])
                follower = BaseFollower(path, base[2], None, step, 0.03, min(config.carry_ramp, step / 5.0))
                if all(scene.collision.is_free(robot.with_base(self._hold, pose), 0.01, load_margin=0.005)
                       for pose in follower.poses()):
                    attempt.retreat = follower
                    break
            scene.collision.release()
            if attempt.retreat is None:
                return self._hold, State.NAVIGATE        # nowhere to back off to: go on from here
        attempt.lost = 0 if is_held(self.env, robot) else attempt.lost + 1
        if attempt.lost >= 4:
            return self._failed("object lost during the back-off")
        target, status = attempt.retreat.step(robot.base_pose(obs["qpos"]))
        if status is Status.FAILED:
            return self._failed(attempt.retreat.failure)
        return robot.with_base(self._hold, target), (State.NAVIGATE if status is Status.DONE else None)

    def _can_carry(self, candidate: GraspCandidate) -> bool:
        """Whether the pads' friction at this grasp holds the object's weight with the safety factor."""
        grip, friction, mass = self._grip(candidate)
        return 2.0 * friction * grip >= self.config.grip_safety * mass * 9.8

    def _grip(self, candidate: GraspCandidate) -> tuple[float, float, float]:
        """Expected force per finger at this grasp (newtons), the friction
        coefficient between pad and object, and the object's mass. The fingers
        squeeze by position control: the force is the finger stiffness times
        how far the finger target lies inside the object, and a finger cannot
        be commanded past fully closed, so a thin wall is held weakly."""
        import pybullet as p
        env, config, scene = self.env, self.config, self.scene
        client = env.pb_physics_client
        mass = p.getDynamicsInfo(env.object_id, -1, physicsClientId=client)[0]
        friction = p.getDynamicsInfo(env.object_id, -1, physicsClientId=client)[1] * \
            p.getDynamicsInfo(env.robot_id, scene.robot.finger_links[0], physicsClientId=client)[1]
        inside = min(config.squeeze, max(0.0, (candidate.width - scene.sim.gripper.min_opening) / 2.0))
        return config.finger_stiffness * inside, friction, mass

    def _cannot_lift(self, candidate: GraspCandidate, obj: ObjectState) -> bool:
        """Whether this grasp cannot lift the object as it lies.

        A grasp away from the centre of mass has to resist the weight's
        turning moment, and the pads can do that only by pressing harder at
        one end of their contact than the other. A weak grip on a short
        contact, far from the centre (a plate held by its rim), cannot."""
        grip, _, mass = self._grip(candidate)
        gripper = self.scene.sim.gripper
        arm = obj.centre - candidate.centre
        moment = np.cross(arm, [0.0, 0.0, -9.8 * mass])
        moment = moment - (moment @ candidate.closing_axis) * candidate.closing_axis   # turning about the pads' line is free
        contact = min(gripper.pad_tip - gripper.pad_back, 2.0 * (gripper.pad_tip - candidate.offset))
        return bool(np.linalg.norm(arm[:2]) >= 0.03 and
                    grip * contact / 2.0 < self.config.moment_safety * np.linalg.norm(moment))

    def _carry_step(self, direction) -> float:
        """Base speed in metres per tick at which the held object does not
        slide across the pads when the base moves along `direction`.

        The env drives the base by position control: within each tick it
        covers most of the gap to its target in the first physics step and
        little in the last, so its speed jumps at every tick. The object
        follows that jump only by friction. (A grasp closing along the travel
        line does not help: at 13.6 mm per tick the 1 kg can swung 25 degrees
        and slid out; at 3 mm per tick it stayed.)"""
        import pybullet as p
        env, config, robot = self.env, self.config, self.scene.robot
        client = env.pb_physics_client
        mass = p.getDynamicsInfo(env.object_id, -1, physicsClientId=client)[0]
        friction = p.getDynamicsInfo(env.object_id, -1, physicsClientId=client)[1] * \
            p.getDynamicsInfo(env.robot_id, robot.finger_links[0], physicsClientId=client)[1]
        grip = sum(c[9] for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=client)
                   if c[3] in robot.finger_links)
        dt = p.getPhysicsEngineParameters(physicsClientId=client)["fixedTimeStep"]
        gain, steps = config.base_gain, int(getattr(env, "substeps", 5))
        # Speed jump at a tick boundary, per metre of step, as an acceleration over one physics step.
        lag = 1.0 / (1.0 - (1.0 - gain) ** steps)
        jolt = gain * lag * (1.0 - (1.0 - gain) ** (steps - 1)) / dt ** 2
        limit = config.carry_safety * friction * grip / (max(mass, 1e-6) * jolt)
        return float(np.clip(limit, config.min_carry_step, config.carry_step))

    def _navigate(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3, not written yet: the solver holds its pose with the
        object in the hand."""
        return self._hold, None

    def _reach(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Move the arm so the object is at the goal.
        Exit: DONE. Failures: object lost, goal out of reach."""
        raise NotImplementedError("segment 3: reach")

    def _retry(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 4. With the object still in the hand, go on carrying it.
        Otherwise start a fresh attempt from where the object now is, if
        enough ticks remain. Exit: NAVIGATE, SETTLE or STOPPED."""
        scene = self._scene(obs)
        if self.attempt.lifted_tick is not None and is_held(self.env, scene.robot):
            return self._hold, State.NAVIGATE
        if self.config.episode_ticks - self.tick < self.config.min_retry_ticks or \
                self.attempt.index + 1 >= self.config.max_attempts:
            return self._hold, State.STOPPED
        unwind = self._way_out(scene)
        self._new_attempt()
        self.attempt.unwind = unwind
        return self._hold, State.SETTLE

    def _way_out(self, scene: Scene) -> Optional[np.ndarray]:
        """After a grasp that got as far as the object: open the hand where it
        is, then go back to where the approach began. The straight approach
        was checked when it was planned; this retraces it."""
        attempt = self.attempt
        if attempt.executor is None or attempt.executor.phase == "transit":
            return None
        fingers = list(scene.robot.fingers)
        opened = self._hold.copy()
        opened[fingers] = scene.robot.upper[fingers]
        back = attempt.grasp_plan.approach[0].copy()
        back[fingers] = scene.robot.upper[fingers]
        back[list(scene.robot.base)] = opened[list(scene.robot.base)]
        return np.array([self._hold.copy(), opened, back])
