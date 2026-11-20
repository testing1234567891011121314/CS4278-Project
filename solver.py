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
from custom_libraries.common import Budget, Status
from custom_libraries.grasping import arm_planner, ik
from custom_libraries.grasping.arm_planner import GraspPlan
from custom_libraries.grasping.candidates import CandidateSettings, GraspCandidate, antipodal_candidates, \
    rake_candidates
from custom_libraries.grasping.executor import ExecutorSettings, GraspExecutor, PathStepper, grasp_error, is_held
from custom_libraries.navigation import carry
from custom_libraries.navigation.base_follower import BaseFollower
from custom_libraries.navigation.grid import OccupancyGrid
from custom_libraries.navigation.parking import (ParkingPose, ParkingSettings, choose_parking, make_follower,
                                                  travel_order)
from custom_libraries.planning_sim import PlanningSim, rotation
from custom_libraries.robot import RobotModel, wrap_angle
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

    # Tick budget of each state (R19). The first three add up to the lift budget.
    settle_ticks: int = 150
    go_to_object_ticks: int = 190
    grasp_ticks: int = 260
    drag_ticks: int = 260
    back_off_ticks: int = 260
    navigate_ticks: int = 2900       # the carry is paced to the ticks left in the episode, which bound it
    reach_ticks: int = 200
    settle_wait_ticks: int = 120     # an object still moving after this is approached as it is
    min_retry_ticks: int = 300       # no new attempt is started with fewer ticks left
    max_attempts: int = 8            # nor after this many attempts

    # Work allowed to each planning call (R26), in steps of work done (common.Budget), never in seconds:
    # a time limit made the same seed pick the plate in one run and not in the next.
    ready_work: int = 6000           # the move to the ready pose
    parking_work: int = 40000        # choosing where the base parks
    planning_work: int = 40000       # the arm motion of one grasp

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
    pad_grip_radius: float = 0.006   # radius at which a pad's friction acts against the object turning on it
    rank_by_mass: bool = False       # True: grasps are ranked by where the centre of mass lies from the pads'
                                     # line and those the weight would swing go last. Withdrawn: a grasp is
                                     # chosen to get the object off the table, not for the carry (docs/plan.md)
    grip_safety: float = 2.0         # the grip's friction must hold this many times the object's weight
    moment_safety: float = 2.0       # the pads must resist this many times the weight's turning moment, or
                                     # the object is first dragged to the edge of what it lies on
    overhang: float = 0.6            # share of the object's half-length left hanging over the edge by a drag
    drag_step: float = 0.006         # base speed while raking, metres per tick: at 20 mm per tick the plate
                                     # kept sliding when the hand stopped and fell off the table
    drag_slack: float = 0.08         # a dragged object may trail the base by this much, metres
    rake_clear: float = 0.35         # an object raked out from under something ends this far beyond its edge
    carry_clearances: tuple = (0.42, 0.38, 0.34)   # distances the carry's route keeps from walls, in order tried
    carry_margin: float = 0.012      # least gap between the robot with its load and the scene during the carry
    aligned_share: float = 0.2       # share of the base's jolt that still acts across the pads when the wrist
                                     # keeps their closing line along the direction of travel
    reach_work: int = 12000          # work allowed to planning the hold for the carry, and the final reach
    # The hold for the carry. The object turns almost freely about the line
    # between the pads, so it is carried hanging below that line.
    carry_hold: bool = True          # re-pose the hand before the carry (False: carry in the post-lift pose)
    level_pads: float = 30.0         # a pads' line tilted more than this from level is rolled level, degrees
    hang_lever: float = 0.04         # a centre of mass this far from the pads' line hangs from it, metres
    askew: float = 20.0              # and this far from straight below it, the hand is laid level, degrees
    hold_reach: float = 0.70         # the hand is pulled in until the object is this far from the base's centre
    swing_lever: float = 1.0         # a centre of mass this far below the pads along the fingers: the wrist
                                     # keeps the pads' line across the travel instead of along it, metres.
                                     # 1.0 = never; 0.06 was used with the hanging sugar box (attempt 10)
    stand_up: float = 2.0            # a hand pointing down at least this steeply (z of its axis) is stood
                                     # upright so the wrist's roll can steer; 2.0 = never. Tested at 0.5 and not
                                     # kept: the object tilts with the hand, its weight then turns it about the
                                     # pads' line, and the can, sugar box and mug were lost sooner
    carry_squeeze: bool = False      # the fingers are commanded fully closed for the carry. Tested and not kept:
                                     # 9 of 15 delivered against 11 without (apple, mug and sugar box squeezed out)
    hold_settle: int = 20            # ticks waited after the re-pose for the object to stop swinging
    hold_rate: float = 0.03          # fastest any arm joint moves while the hold is taken, rad per tick
    hang_long: bool = False          # withdrawn, as rank_by_mass. True: a long object lying down is gripped between its centre of mass and its
                                     # nearer end, so it hangs on end from the pads with that end (a hammer's
                                     # head) above the fingers: it does not lag the base's turns, and if it
                                     # creeps down through the pads the head comes to rest on them
    long_length: float = 0.25        # "long": at least this many metres
    long_ratio: float = 2.0          # and this many times as long as wide
    long_mass: float = 0.0           # and at least this heavy, kg. With 0.17 m, 1.7 and 0.3 kg the rule also
                                     # takes the sugar box, which then hangs and is carried 18.7 m of 21.4
                                     # before it creeps out of the pads; not kept (NAVIGATE.md, attempt 10)
    hang_grip: float = 0.035         # least distance along a long object from the grip to the centre of mass
    carry_reserve: int = 300         # ticks kept back from the carry for its setup (raise, turn, rolls) and the reach
    carry_haste: float = 1.25        # the object is led this many times faster than the ticks left require
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
    grid: OccupancyGrid                 # for the drive to the object: the base may reach under a table
    carry_grids: list[OccupancyGrid]    # for the carry: tables are obstacles; one grid per distance kept from walls


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
    carry: Optional[carry.CarryFollower] = None
    carry_failures: int = 0                         # carries or reaches that failed with the object still held
    hold: Optional[PathStepper] = None              # the move into the hold for the carry
    hold_wait: int = 0
    reach: Optional[PathStepper] = None
    reach_wait: int = 0
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
            carry_grids = [OccupancyGrid.from_world(world, self.config.grid_resolution, keep, obstacle_radius=keep)
                           for keep in self.config.carry_clearances]
            self.scene = Scene(robot=sim.robot, sim=sim, collision=CollisionChecker(sim), world=world, grid=grid,
                               carry_grids=carry_grids)
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
        budget = Budget(config.ready_work)
        for _ in range(4):
            ready = ik.solve(scene.sim, position, turn, opened, self.rng, budget)
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
                                                Budget(self.config.ready_work),
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
                                         limit=config.max_found,
                                         settings=CandidateSettings(central="mass" if config.rank_by_mass
                                                                    else "centre"))
            strong = [c for c in found if self._can_carry(c)]
            fit = arm_planner.hands_that_fit(strong, obj, scene.sim, scene.collision)
            # A long object lying down lags every turn of the base if it is
            # held level. Gripped across its length, between the centre of
            # mass and the nearer end, it hangs on end from the pads with
            # that end up: those grasps go first.
            axis = obj.long_axis(config.long_length, config.long_ratio) if config.hang_long else None
            if axis is not None and (abs(axis[2]) > 0.5 or self._grip(found[0])[2] < config.long_mass if found
                                     else True):
                axis = None                                         # standing on end already, or light
            if axis is not None:
                ends = (obj.cloud - obj.position) @ axis
                if ends.max() > -ends.min():
                    axis = -axis                                    # `axis` points from the centre of mass to the nearer end
                fit = [c for c in fit if self._hangs(c, obj, axis)] + [c for c in fit if not self._hangs(c, obj, axis)]
            candidates = fit[:config.max_candidates]
            # Tiers, tried in this order whatever the scores: grasps that can
            # resist the weight's turning moment before those that cannot;
            # grasps the weight will not swing about the pads' line before
            # those it will (for a long object lying down, grasps it hangs
            # from before the rest);
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
                tiers.append(16 * tried + 8 * self._cannot_lift(c, obj) + 4 * ((not self._hangs(c, obj, axis)) if axis is not None else
                                  (config.rank_by_mass and self._swings(c, obj))) +
                             2 * (height > min(under) + 0.05) + 1 * shallow)
            # If nothing can lift the object as it lies on a table, rake it
            # to the table's edge first: there the hand can reach round it.
            rake = attempt.pick_mode == "table" and not self._dragged and all(t & 8 for t in tiers)
            toward = scene.robot.base_pose(self._hold)[:2] - obj.centre[:2]
            if rake:
                candidates = rake_candidates(obj, scene.sim.gripper, support, toward)
                tiers = [0] * len(candidates)
            if not candidates:
                attempt.search = dict(found=len(found), candidates=0)
                return self._failed("no grasp candidates")
            attempt.candidates, attempt.tiers = candidates, tiers
            attempt.object_at_plan = obj.position.copy()
            work = Budget(config.parking_work)
            attempt.parking, stats = choose_parking(attempt.candidates, obj, scene.grid, scene.sim,
                                                    scene.collision, self._hold, self.rng, work,
                                                    exclude=tuple(self._tried_parking),
                                                    facing=toward if rake else None, tiers=tiers)
            stats["work"] = work.spent
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
                                                            Budget(config.parking_work), facing=toward)
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
            work = Budget(config.planning_work)
            plan, stats = arm_planner.plan_grasp([attempt.candidates[k] for k in order], self._hold, obj,
                                                 scene.sim, scene.collision, self.rng, work,
                                                 drag_for=lambda c: c.kind == "rake")
            stats["work"] = work.spent
            attempt.plan_stats = stats
            if plan is None:
                if attempt.parking is not None:
                    self._tried_parking.append(attempt.parking.pose[:2])
                return self._failed("no grasp plan")
            attempt.grasp_plan = plan
            c = plan.candidate
            scene.sim.set_object(obj.position, obj.orientation)
            stats["wedge_deg"] = round(float(np.degrees(scene.sim.wedge(*c.gripper_pose(), c.opening))), 1)
            stats["grasp"] = dict(kind=c.kind, width_mm=round(1000 * c.width, 1), opening_mm=round(1000 * c.opening, 1),
                                  offset_mm=round(1000 * c.offset, 1), approach=np.round(c.approach_axis, 2).tolist(),
                                  closing=np.round(c.closing_axis, 2).tolist(), lift_m=round(plan.lift_height, 3))
            attempt.executor = GraspExecutor(plan, self.env, scene.sim, config.lift_height, config.hold_ticks,
                                             ExecutorSettings(squeeze=config.squeeze))
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
        arm = (obj.position if self.config.rank_by_mass else obj.centre) - candidate.centre
        moment = np.cross(arm, [0.0, 0.0, -9.8 * mass])
        moment = moment - (moment @ candidate.closing_axis) * candidate.closing_axis   # turning about the pads' line is free
        contact = min(gripper.pad_tip - gripper.pad_back, 2.0 * (gripper.pad_tip - candidate.offset))
        return bool(np.linalg.norm(arm[:2]) >= 0.03 and
                    grip * contact / 2.0 < self.config.moment_safety * np.linalg.norm(moment))

    def _swings(self, candidate: GraspCandidate, obj: ObjectState) -> bool:
        """Whether the object's weight would turn it in this grasp. Nothing
        but the pads' friction on a small patch resists turning about the
        line between them, so a centre of mass to one side of that line swings
        the object round until it hangs below it."""
        grip, friction, mass = self._grip(candidate)
        off = obj.position - candidate.centre
        off = off - (off @ candidate.closing_axis) * candidate.closing_axis     # from the line to the centre of mass
        level = np.cross(candidate.closing_axis, [0.0, 0.0, 1.0])               # horizontal, across the line
        length = np.linalg.norm(level)
        sideways = abs(float(off @ level)) / length if length > 1e-6 else 0.0
        return bool(9.8 * mass * sideways > 2.0 * friction * grip * self.config.pad_grip_radius)

    def _hangs(self, candidate: GraspCandidate, obj: ObjectState, axis: np.ndarray) -> bool:
        """Whether a long object would hang on end from this grasp, its
        nearer end up. `axis` points along its length from the centre of mass
        to that end: the pads close across the length, on that side of the
        centre of mass."""
        along = float((candidate.centre - obj.position) @ axis)
        return bool(abs(float(candidate.closing_axis @ axis)) < 0.35 and along >= self.config.hang_grip)

    def _carry_step(self, direction) -> float:
        """Base speed for the back-off, metres per tick."""
        return float(np.clip(self._friction_step(), self.config.min_carry_step, self.config.carry_step))

    def _friction_step(self) -> float:
        """Base speed in metres per tick at which the held object does not
        slide across the pads.

        The env drives the base by position control: within each tick it
        covers most of the gap to its target in the first physics step and
        little in the last, so its speed jumps at every tick. The object
        follows that jump only by friction. (A grasp closing along the travel
        line does not help by itself: at 13.6 mm per tick the 1 kg can swung
        25 degrees and slid out; at 3 mm per tick it stayed.)"""
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
        return float(config.carry_safety * friction * grip / (max(mass, 1e-6) * jolt))

    def _navigate(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Carry the object through the maze: the object is led
        along the route, the base trailing an arm's length behind (see
        navigation/carry.py). Exit: REACH. Failures: no clear route, object
        lost, base blocked."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        robot = scene.robot
        if config.carry_hold and attempt.carry is None and attempt.hold_wait < config.hold_settle:
            # First the hold for the carry: the hand is re-posed, slowly.
            if attempt.hold is None:
                limit = np.full(len(self._hold), config.hold_rate)
                limit[robot.torso] = 0.002
                path = self._plan_hold(obs)
                if config.carry_squeeze:
                    fingers = list(robot.fingers)
                    path[:, fingers] = robot.lower[fingers]
                    if len(path) < 2:
                        path = np.array([self._hold, path[0]])       # only the fingers change
                attempt.hold = PathStepper(path, limit)
                if len(attempt.hold.path) < 2:
                    attempt.hold_wait = config.hold_settle           # nothing to change: no wait
            attempt.lost = 0 if is_held(self.env, robot) else attempt.lost + 1
            if attempt.lost >= 4:
                return self._failed("object lost while taking the hold for the carry")
            if not attempt.hold.finished:
                return attempt.hold.step(), None
            attempt.hold_wait += 1
            return self._hold, None
        if attempt.carry is None:
            attempt.carry = self._plan_carry(obs)
            if attempt.carry is None:
                return self._failed("no clear route to the goal")
        attempt.lost = 0 if is_held(self.env, robot) else attempt.lost + 1
        if attempt.lost >= 4:
            return self._failed("object lost during the carry")
        action, status = attempt.carry.step(robot.base_pose(obs["qpos"]))
        if status is Status.FAILED:
            return self._failed(attempt.carry.failure)
        return action, (State.REACH if status is Status.DONE else None)

    def _grip_point(self) -> Optional[np.ndarray]:
        """Where the pads hold the object: the middle of the fingers' contacts, weighted by force."""
        import pybullet as p
        env, robot = self.env, self.scene.robot
        points = [(np.array(c[6]), c[9]) for c in
                  p.getContactPoints(env.robot_id, env.object_id, physicsClientId=env.pb_physics_client)
                  if c[3] in robot.finger_links and c[9] > 0]
        weight = sum(force for _, force in points)
        return sum(point * force for point, force in points) / weight if weight > 0 else None

    def _plan_hold(self, obs) -> np.ndarray:
        """The (K, 15) move from the post-lift pose into the hold for the
        carry (see `_hold_path`). If the hand cannot be laid level, the arm
        is still folded with the hand as it is."""
        path = self._hold_path(obs, lay=True)
        stats = self.attempt.plan_stats["hold"]
        if len(path) < 2 and "laid_deg" in stats:
            path = self._hold_path(obs, lay=False)
            self.attempt.plan_stats["hold"] = dict(self.attempt.plan_stats["hold"], not_laid=stats.get("blocked", "no pose"))
        return path

    def _hold_path(self, obs, lay: bool) -> np.ndarray:
        """The (K, 15) move from the post-lift pose into the hold for the carry.

        The object turns almost freely about the line between the pads, and
        the arm as it lifted the object sticks out a third of a metre to one
        side of the base. In the hold:
        1. A pads' line that is not level (a plate held flat by its rim) is
           rolled level, the centre of mass below it: the object hangs from
           the pads instead of sticking out from them.
        2. Where the object hangs askew from a level line with the hand
           pointing down (a hammer gripped along its handle), the hand is laid
           level, pointing away from the side the weight hangs toward, so the
           part above the pads passes between the fingers instead of into the
           palm and the object hangs on end.
        3. The hand comes in to the base's centre line, the arm folded in the
           vertical plane through it: the robot is then as narrow as its base.
        If no such pose is within reach and clear, the post-lift pose is kept."""
        scene, config = self.scene, self.config
        robot, sim = scene.robot, scene.sim
        q0 = self._hold.copy()
        position, turn = sim.gripper_pose(q0)
        approach, closing = turn[:, 0], turn[:, 1]
        centre = np.asarray(obs["object_pos"], dtype=float)
        grip = self._grip_point()
        if grip is None:
            grip = position + approach * (sim.gripper.pad_back + sim.gripper.pad_tip) / 2.0
        lever = centre - grip
        base = robot.base_pose(q0)
        forward = np.array([np.cos(base[2]), np.sin(base[2]), 0.0])
        left = np.array([-forward[1], forward[0], 0.0])
        up = np.array([0.0, 0.0, 1.0])
        stats, rigid, spin = {}, True, np.eye(3)

        def heading(v) -> float:
            return float(np.arctan2(v[1], v[0]))

        # 1. Level the pads' line: of the two rolls that do, the one that leaves the centre of mass lower.
        if abs(closing[2]) > np.sin(np.radians(config.level_pads)):
            tilt = np.arctan2(closing[2], turn[2, 2])
            rolls = [float(wrap_angle(-tilt + k * np.pi)) for k in (0, 1)]
            roll = min(rolls, key=lambda a: (ik.axis_rotation(approach, a) @ lever)[2])
            spin = ik.axis_rotation(approach, roll)
            stats["rolled_deg"] = round(float(np.degrees(roll)))
        pointing, line, weight = spin @ approach, spin @ closing, spin @ lever
        hangs = weight - (weight @ line) * line                     # from the pads' line to the centre of mass
        askew = np.degrees(np.arccos(np.clip(-hangs[2] / max(np.linalg.norm(hangs), 1e-9), -1.0, 1.0)))
        if lay and np.linalg.norm(hangs) > config.hang_lever and abs(pointing[2]) > 0.7 and askew > config.askew:
            # 2. Turn about the vertical until the weight hangs toward the
            # base, then lay the hand level about the pads' line, pointing forward.
            yaw = ik.axis_rotation(up, heading(-forward) - heading(hangs))
            line, pointing = yaw @ line, yaw @ pointing
            across = np.cross(line, pointing)
            pitch = np.arctan2(-pointing[2], across[2])
            pitch = min((float(wrap_angle(pitch + k * np.pi)) for k in (0, 1)),
                        key=lambda a: -float((ik.axis_rotation(line, a) @ pointing) @ forward))
            spin = ik.axis_rotation(line, pitch) @ yaw @ spin
            rigid = False                                           # the object hangs: it does not pitch with the hand
            carried = np.array([0.0, 0.0, -np.linalg.norm(hangs)])
            stats["laid_deg"] = round(float(np.degrees(pitch)))
        else:
            tilt = np.cross(pointing, -up)
            if pointing[2] < -config.stand_up and np.linalg.norm(tilt) > 1e-6:
                # 3a. A hand that points mostly down is stood straight up,
                # the object tilting with it: only then can the wrist's roll
                # keep the pads' line along the travel.
                lean = float(np.arcsin(min(1.0, np.linalg.norm(tilt))))
                spin = ik.axis_rotation(tilt / np.linalg.norm(tilt), lean) @ spin
                stats["stood_up_deg"] = round(float(np.degrees(lean)))
            elif np.linalg.norm(pointing[:2]) > 0.3:
                # 3b. Any other hand is turned to point forward.
                spin = ik.axis_rotation(up, heading(forward) - heading(pointing)) @ spin
            carried = spin @ lever
        reach = min(float(np.linalg.norm(centre[:2] - base[:2])), config.hold_reach)
        grip_to = np.array([base[0], base[1], grip[2]]) + reach * forward - [carried[0], carried[1], 0.0]
        turn_to = spin @ turn
        position_to = grip_to + spin @ (position - grip)

        scene.collision.focus(base[:2])
        scene.collision.hold(q0, centre, obs["object_xyzw"])
        try:
            def clear(q) -> bool:
                # The env does not collide the robot with itself, so the arm
                # may pass close to the body; the object may not.
                gaps = scene.collision.clearances(q, with_object=rigid)
                return gaps["scene"] >= 0.01 and gaps["own"] >= 0.0 and \
                    (not rigid or (gaps["load"] >= 0.005 and gaps["object"] >= 0.005))

            def spread(q) -> float:
                """How far the arm sticks out to the side of the base's centre line, metres."""
                sim.set_robot(q)
                return max(abs(float((sim.link_pose(robot.joint_indices[k])[0] - [base[0], base[1], 0.0]) @ left))
                           for k in robot.arm)

            # The end pose, from seeds with the arm in the plane of the centre line.
            budget, found = Budget(config.reach_work), []
            pan, lift, upper_roll, elbow, fore_roll, flex, _ = robot.arm
            for seed_lift in (-0.8, -0.2, 0.4):
                for seed_elbow in (0.8, 1.6):
                    for seed_flex in (-0.8, 0.4, 1.4):
                        seed = q0.copy()
                        seed[[pan, upper_roll, fore_roll]] = 0.0
                        seed[[lift, elbow, flex]] = seed_lift, seed_elbow, seed_flex
                        q = ik.solve(sim, position_to, turn_to, seed, self.rng, budget, restarts=0, unstick=True)
                        if q is not None and clear(q):
                            found.append(q)
            if not found:
                stats["kept"] = "no folded pose within reach and clear"
                return np.array([q0])
            joints = list(robot.plan_joints)
            vector = ik.rotation_vector(spin)
            angle = float(np.linalg.norm(vector))
            axis = vector / angle if angle > 1e-9 else up

            def leg(start, end, turned: tuple, moved: tuple, name: str) -> Optional[list]:
                """Joint values from `start` to `end` along which the hand
                turns from `turned[0]` to `turned[1]` of the whole turn and
                moves from `moved[0]` to `moved[1]` of the whole way, the
                arm's fold following the joint values in between. None if
                the arm cannot follow or the way is blocked."""
                swing = angle * abs(turned[1] - turned[0])
                count = max(2, int(np.ceil(max(np.abs(end[joints] - start[joints]).max() / 0.05, swing / 0.05))))
                out, last = [], np.array(start, dtype=float)
                for k in range(1, count + 1):
                    t = k / count
                    part = ik.axis_rotation(axis, (turned[0] + t * (turned[1] - turned[0])) * angle)
                    shift = (moved[0] + t * (moved[1] - moved[0])) * (grip_to - grip)
                    # Each step continues from the one before, nudged toward
                    # the end's fold, so the arm never jumps between solutions.
                    q = ik.solve(sim, grip + shift + part @ (position - grip), part @ turn,
                                 last + (end - start) / count, self.rng, budget, restarts=0, unstick=True)
                    if q is None or np.abs(q[joints] - last[joints]).max() > 0.3:
                        stats["blocked"] = f"{name}, step {k} of {count}: " + \
                            ("out of reach" if q is None else "the arm would jump")
                        return None
                    last = q
                    if not clear(q):
                        gaps = scene.collision.clearances(q, with_object=rigid)
                        stats["blocked"] = f"{name}, step {k} of {count}: " + \
                            ", ".join(f"{part} {1000 * gap:.0f} mm" for part, gap in gaps.items() if gap < 0.011)
                        return None
                    out.append(q)
                return out

            def way(q1) -> Optional[list]:
                """The way to the end pose `q1`: the turn and the move at
                once; failing that, the turn where the hand is, then the move."""
                both = leg(q0, q1, (0.0, 1.0), (0.0, 1.0), "turning while folding")
                if both is not None:
                    return [q0] + both
                turned = leg(q0, q0, (0.0, 1.0), (0.0, 0.0), "turning in place")
                if turned is None:
                    return None
                folded = leg(turned[-1], q1, (1.0, 1.0), (0.0, 1.0), "folding after the turn")
                return None if folded is None else [q0] + turned + folded

            # The narrowest end pose whose way in is clear.
            for q1 in sorted(found, key=spread)[:6]:
                q1 = q1.copy()
                q1[joints] = np.where(robot.wraps[joints], q0[joints] + wrap_angle(q1[joints] - q0[joints]),
                                      q1[joints])
                path = way(q1)
                if path is not None:
                    stats.update(spread_cm=[round(100 * spread(q0)), round(100 * spread(q1))],
                                 reach_m=round(reach, 2))
                    return np.array(path)
            stats["kept"] = "the way into every folded pose is blocked"
            return np.array([q0])
        finally:
            scene.collision.release()
            self.attempt.plan_stats = dict(self.attempt.plan_stats or {}, hold=stats)

    def _plan_carry(self, obs) -> Optional[carry.CarryFollower]:
        """Choose a route and check the whole carry along it, the object in
        the hand, against the scene in the planning copy."""
        import pybullet as p
        scene, config, env = self.scene, self.config, self.env
        robot, world = scene.robot, scene.world
        settings = carry.CarrySettings()
        base = robot.base_pose(self._hold)
        position = np.asarray(obs["object_pos"], dtype=float)
        state = p.getLinkState(env.robot_id, robot.gripper_link, computeForwardKinematics=True,
                               physicsClientId=env.pb_physics_client)
        hand = rotation(state[5])
        upright = abs(hand[2, 0]) >= settings.upright         # the hand's axis is close to vertical
        closing = float(np.arctan2(hand[1, 1], hand[0, 1])) if upright else None
        # Which line the wrist keeps along the travel. An object held at its
        # centre of mass is pushed by a pad if the pads' line is along the
        # travel. One whose weight hangs well below the pads (a tall bottle
        # held by its top) is rocked out of the grip by that push; with the
        # pads' line across the travel it swings about the line instead.
        grip = self._grip_point()
        below = float((position - grip) @ hand[:, 0]) if grip is not None else 0.0
        swings = upright and below > config.swing_lever
        if swings:
            closing += np.pi / 2.0
        share = config.aligned_share if upright else 1.0
        safe = self._friction_step() / share
        ticks_left = config.episode_ticks - self.tick - config.carry_reserve
        # The torso takes the object toward the goal's height; the reach does the rest.
        room = (robot.lower[robot.torso] + 0.005 - self._hold[robot.torso],
                robot.upper[robot.torso] - 0.005 - self._hold[robot.torso])
        rise = float(np.clip(world.goal[2] - position[2], min(room[0], 0.0), max(room[1], 0.0)))
        raised = self._hold.copy()
        raised[robot.torso] += rise
        roll_joint = robot.arm[-1]
        scene.collision.hold(self._hold, position, obs["object_xyzw"])
        blocked = []
        try:
            for route in carry.plan_route(scene.carry_grids, base[:2], world.goal[:2], settings):
                # Slower is safer for the hold, so the object is led only a
                # little faster than the ticks that are left require.
                arm = float(np.linalg.norm(position[:2] - base[:2]))
                length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
                needed = (length - arm) / max(ticks_left, 1)
                step = float(np.clip(config.carry_haste * needed, settings.min_step, settings.max_step))
                plan = carry.make_plan(route, base, position[:2], closing, self._hold[roll_joint],
                                       int(np.sign(hand[2, 0])), step)
                if plan.length <= plan.arm:
                    continue
                other = plan.turn - np.sign(plan.turn) * 2.0 * np.pi if abs(plan.turn) > 1e-6 else plan.turn
                for turn in dict.fromkeys([plan.turn, other]):          # the short way round, then the long way
                    clear = True
                    for index, (pose, roll) in enumerate(carry.poses_to_check(plan, turn)):
                        q = robot.with_base(raised, pose)
                        q[roll_joint] = roll
                        scene.collision.focus(pose[:2])
                        if not scene.collision.is_free(q, config.carry_margin, load_margin=config.carry_margin):
                            gaps = scene.collision.clearances(q, with_object=False)
                            blocked.append(dict(pose=index, turn_deg=round(float(np.degrees(turn))),
                                                base=[round(float(v), 2) for v in pose],
                                                nearest=scene.collision.nearest(q)[1],
                                                gaps_mm={k: round(1000 * v) for k, v in gaps.items()}))
                            clear = False
                            break
                    if clear:
                        self.attempt.plan_stats = dict(self.attempt.plan_stats or {},
                                                       carry=dict(route_m=round(plan.length, 2),
                                                                  step_mm=round(1000 * step, 1),
                                                                  safe_mm=round(1000 * safe, 1), steered=bool(upright),
                                                                  across=bool(swings), below_mm=round(1000 * below),
                                                                  rise_m=round(rise, 3),
                                                                  turn_deg=round(float(np.degrees(turn)))))
                        return carry.CarryFollower(plan, robot, self._hold, rise, turn, settings)
            self.attempt.plan_stats = dict(self.attempt.plan_stats or {}, carry_blocked=blocked[:4])
            return None
        finally:
            scene.collision.release()

    def _reach(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Move the hand so the object, not the hand, is at the
        goal. Exit: DONE. Failures: object lost, goal out of reach."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        robot = scene.robot
        gap = scene.world.goal - np.asarray(obs["object_pos"], dtype=float)
        distance = float(np.linalg.norm(gap))
        attempt.lost = 0 if is_held(self.env, robot) else attempt.lost + 1
        if attempt.lost >= 4:
            return self._failed("object lost during the reach")
        if attempt.reach is None:
            if distance < 0.5 * config.goal_tolerance:
                self.report.delivered_tick = self.tick
                return self._hold, State.DONE
            scene.collision.hold(self._hold, obs["object_pos"], obs["object_xyzw"])
            scene.collision.focus(robot.base_pose(self._hold)[:2])
            why = {}
            path = arm_planner.straight_line(self._hold, gap / distance, distance, scene.sim, scene.collision,
                                             self.rng, Budget(config.reach_work), arm_planner.PlannerSettings(),
                                             load_after=0.0, least=max(0.01, distance - 0.5 * config.goal_tolerance),
                                             why=why, unstick=True)
            scene.collision.release()
            if path is None or len(path) < 2:
                return self._failed(f"goal out of reach ({why.get('stopped')})")
            limit = np.full(len(self._hold), 0.02)
            limit[robot.torso] = 0.004
            attempt.reach = PathStepper(path, limit)
        if not attempt.reach.finished:
            return attempt.reach.step(), None
        attempt.reach_wait += 1
        if distance < config.goal_tolerance * 0.8 or (attempt.reach_wait > 20 and distance < config.goal_tolerance):
            self.report.delivered_tick = self.tick
            return self._hold, State.DONE
        if attempt.reach_wait > 40:
            return self._failed(f"object stopped {distance:.3f} m from the goal")
        return self._hold, None

    def _retry(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 4. With the object still in the hand, go on carrying it
        (the carry and the reach are planned afresh).
        Otherwise start a fresh attempt from where the object now is, if
        enough ticks remain. Exit: NAVIGATE, SETTLE or STOPPED."""
        scene = self._scene(obs)
        if self.attempt.lifted_tick is not None and is_held(self.env, scene.robot):
            # Carry on from here with a fresh plan, unless that has already failed twice.
            attempt = self.attempt
            attempt.carry, attempt.reach, attempt.reach_wait, attempt.lost = None, None, 0, 0
            attempt.carry_failures += 1
            return self._hold, (State.NAVIGATE if attempt.carry_failures <= 2 else State.STOPPED)
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
