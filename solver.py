"""Pick-Nav-Reach solver: one state machine, one action per tick.

Drop-in for the course's KeyBoardController (R1):

    solver = Solver(env)
    while True:
        obs, info = env.step(solver.get_action())

See docs/plan.md for the segment each state belongs to. The pick (SETTLE to
the end of GRASP and DRAG) is that of commit b701e73 and is not changed for
the carry's sake. Everything after it (BACK_OFF, NAVIGATE, REACH) is planned
in custom_libraries/navigation/transport.py.
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
from custom_libraries.grasping.candidates import GraspCandidate, antipodal_candidates, rake_candidates
from custom_libraries.grasping.executor import GraspExecutor, PathStepper, grasp_error, is_held
from custom_libraries.navigation import carry, transport
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

    # Tick budget of each state (R19). The first three add up to the lift budget.
    settle_ticks: int = 150
    go_to_object_ticks: int = 190
    grasp_ticks: int = 260
    drag_ticks: int = 260
    back_off_ticks: int = 260
    navigate_ticks: int = 2900       # the carry is paced to the ticks left in the episode, which bound it
    reach_ticks: int = 600           # the final approach, a raise to the goal's height, a second approach
    settle_wait_ticks: int = 120     # an object still moving after this is approached as it is
    min_retry_ticks: int = 300       # no new attempt is started with fewer ticks left
    max_attempts: int = 8            # nor after this many attempts

    # Work allowed to each planning call (R26), in cycles of work done (common.Budget), never in seconds:
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
    grip_safety: float = 2.0         # the grip's friction must hold this many times the object's weight
    moment_safety: float = 2.0       # the pads must resist this many times the weight's turning moment, or
                                     # the object is first dragged to the edge of what it lies on
    overhang: float = 0.6            # share of the object's half-length left hanging over the edge by a drag
    drag_step: float = 0.006         # base speed while raking, metres per tick: at 20 mm per tick the plate
                                     # kept sliding when the hand stopped and fell off the table
    drag_slack: float = 0.08         # a dragged object may trail the base by this much, metres
    rake_clear: float = 0.35         # an object raked out from under something ends this far beyond its edge
    # After the pick (custom_libraries/navigation/transport.py). Planning work is counted in cycles, as above.
    back_off_headings: tuple = (0.0, 45.0, -45.0, 90.0, -90.0, 135.0, -135.0, 180.0)
                                     # directions tried for the back-off, degrees from straight away from
                                     # where the object stood
    back_off_distances: tuple = (0.15, 0.3, 0.45, 0.6)   # lengths tried, metres, after "no retreat at all":
                                     # straight away at each length first, then the other directions
    back_off_tries: int = 16         # retreats examined before falling back to `back_off` straight away
    back_off_look: float = 1.5       # metres of the route checked to tell whether a carry can start somewhere
    travel_upright: float = 0.8      # a hand pointing down at least this steeply (-z of its axis) is stood
                                     # straight down for the carry; any other is kept and turned forward
    travel_heavy: float = 0.6        # from this mass, kg, a hand pointing down is laid level first: the object
                                     # then slides into the palm, not out along the fingers
    travel_order: tuple = ()         # experiments: a fixed order of travel poses in place of the rule, names
                                     # out of "under", "level", "down", "folded" (transport.travel_poses)
    travel_reach: float = 0.60       # the object is brought in to this far from the base's centre, metres. At
                                     # 0.70 the level poses' elbow met the door jamb of the maze on the way in
                                     # and six of fifteen carries were refused; at 0.50 the arm cannot take them
    travel_arms: int = 3             # arms tried for each travel pose, the narrowest first, until the carry passes
    travel_more_ways: int = 2        # ways into a travel pose tried when the hand cannot be led there straight:
                                     # 0 none, 1 the move before the turn, 2 then also joint by joint
                                     # (travel_pose.fold_path); a weak hold is never led joint by joint
    travel_hand_step: float = 0.006  # fastest the hand moves along a way made joint by joint, metres per tick
    free_gap: float = 0.03           # a held object that rests against something is first taken this far off it
    free_reach: float = 0.20         # ... by a straight move of the hand of at most this length, metres (0: never)
    travel_work: int = 12000         # work allowed to planning the move into the travel pose
    travel_rate: float = 0.03        # fastest any arm joint moves while the travel pose is taken, rad per tick
    travel_settle: int = 20          # ticks waited after it for the object to stop swinging
    carry_clearances: tuple = (0.42, 0.38, 0.34)   # distances the carry's route keeps from walls, in order tried
    carry_margin: float = 0.012      # least gap between the robot with its load and the scene during the carry
    carry_reserve: int = 300         # ticks kept back from the carry for its set-up (raise, turn, rolls) and the reach
    carry_haste: float = 1.25        # the object is led this many times faster than the ticks left require
    carry_fragile: float = 0.0045    # a hold whose own speed limit (_carry_step, metres per tick) is under this
                                     # gets no haste at all and no way into a pose made joint by joint (0: never).
                                     # The three objects under it (can 3.0, bleach 3.2, plate 3.3 to 3.9 mm per
                                     # tick) were the three never delivered on cloud seeds 2 and 3
    reseat_slack: int = 150          # after a re-seat the pace is set so the carry ends this many ticks early
    carry_turn_step: float = 0.006   # fastest the object goes round in the turn on the spot, metres per tick
    carry_max_yaw: float = 0.05      # most the base turns per tick during the drive, rad: the object is slowed
                                     # in a bend until the turn is within it
    reseat_sag: float = 0.035        # an object that has sunk this far below the axis of a level hand, across
                                     # the pads, is re-seated by half a turn of the wrist, metres (0: never)
    reseat_room: float = 0.04        # least gap to everything fixed, all through that turn
    reseat_lever: float = 0.03       # ... and only where the object's centre is within this of the pads along the
                                     # hand's axis: held by one end it droops instead, and the turn does not help
    carry_retries: int = 2           # times a carry that failed with the object still in the hand is planned again
    carry_retry_margins: tuple = (1.0, 0.5, 0.15)   # share of carry_margin asked for by the first plan and by
                                     # each one after it: the same plan a second time is refused a second time
    lost_ticks: int = 4              # ticks in a row out of the hand that count as a drop
    reach_work: int = 12000          # work allowed to planning the final reach
    reach_step: float = 0.004        # fastest the base moves the object in the final approach, metres per tick
    reach_near: float = 0.25         # an object this close to the goal on the floor plan goes straight to the reach
    reach_approach_ticks: int = 150  # ticks the base and torso may take over the final approach
    raise_reserve: int = 200         # ticks kept back from the carry of an object that will arrive under the
                                     # goal, for raising it there
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
    away: bool = False                              # the back-off has been planned: the hand has left the grasp
    freed: bool = False                             # the held object has been looked at for what it rests against
    freeing: Optional[PathStepper] = None           # the move that takes it off that
    travel: Optional[PathStepper] = None            # the move into the travel pose
    travel_wait: int = 0                            # ticks waited after it
    carry: Optional[carry.CarryFollower] = None     # the drive to the goal
    carry_failures: int = 0                         # carries that failed with the object still in the hand
    sag_from: Optional[float] = None                # least the object has been below a level hand's axis, metres
    fragile: Optional[bool] = None                  # the hold's own speed limit is under Config.carry_fragile
    seatable: bool = False                          # this carry's hold can be re-seated by half a turn of the wrist
    reseats: int = 0                                # re-seats of this carry that have been paced for
    reach: Optional[PathStepper] = None             # the final move of the hand
    reach_wait: int = 0
    raising: Optional[PathStepper] = None           # the move that brings the object up to the goal's height
    raised: bool = False                            # that move has been tried
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
        self._carry_grids: Optional[list] = None         # floor maps for the carry's route
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
                                                    Budget(config.parking_work),
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
            plan, stats = arm_planner.plan_grasp([attempt.candidates[k] for k in order], self._hold, obj,
                                                 scene.sim, scene.collision, self.rng,
                                                 Budget(config.planning_work),
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
        """Segment 3. Take the base, with the object, to where the carry can
        start. The retreat is chosen from the scene (transport.choose_back_off):
        its direction and length, or none at all. It is driven slowly, with
        the arm still. Exit: NAVIGATE. Failures: object lost, base blocked."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        robot = scene.robot
        if not attempt.freed:
            # An object picked up where it lay can still rest against the
            # floor or a wall: nothing can be planned until it is off that.
            attempt.freed = True
            push = transport.load_contact(self.env) if config.free_reach > 0 else None
            if push is not None:
                path, stats = transport.plan_free(scene, self._hold, self._load(obs), push, config)
                attempt.plan_stats = dict(attempt.plan_stats or {}, freed=stats)
                if path is not None:
                    limit = np.full(len(self._hold), config.travel_rate)
                    limit[robot.torso] = 0.002
                    attempt.freeing = PathStepper(path, limit)
        if attempt.freeing is not None:
            if self._dropped(obs):
                return self._failed("object lost while it was taken off what it rested against")
            if not attempt.freeing.finished:
                return attempt.freeing.step(), None
            attempt.freeing = None
        if not attempt.away:
            attempt.away = True
            base = robot.base_pose(self._hold)
            origin = attempt.object_at_plan if attempt.object_at_plan is not None else obs["object_pos"]
            away = base[:2] - np.asarray(origin, dtype=float)[:2]
            step = self._carry_step(away)
            attempt.retreat, stats = transport.choose_back_off(
                scene, self._grids(scene), self.rng, self._hold, self._load(obs), away, step,
                min(config.carry_ramp, step / 5.0), config.episode_ticks - self.tick, config)
            attempt.plan_stats = dict(attempt.plan_stats or {}, carry_step_mm=round(1000 * step, 1), back_off=stats)
            if attempt.retreat is None:
                return self._hold, State.NAVIGATE        # the carry can start from here
        if self._dropped(obs):
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

    def _grids(self, scene: Scene) -> list:
        if self._carry_grids is None:
            self._carry_grids = transport.carry_grids(scene, self.config)
        return self._carry_grids

    def _load(self, obs) -> transport.Load:
        """The object as it is held now, for the planners after the pick."""
        import pybullet as p
        grip, _ = transport.grip_point(self.env, self.scene.robot)
        mass = p.getDynamicsInfo(self.env.object_id, -1, physicsClientId=self.env.pb_physics_client)[0]
        return transport.Load(self.scene, self._hold, obs["object_pos"], obs["object_xyzw"], grip, mass)

    def _dropped(self, obs) -> bool:
        """True once the object has been out of the hand `lost_ticks` ticks running."""
        attempt = self.attempt
        attempt.lost = 0 if transport.in_hand(self.env, self.scene.robot, obs) else attempt.lost + 1
        return attempt.lost >= self.config.lost_ticks

    def _navigate(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Take the travel pose, slowly; then carry the object
        through the maze, the object led along the route and the base turning
        with it an arm's length behind (navigation/carry.py). Exit: REACH.
        Failures: object lost, no clear route, base blocked."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        robot = scene.robot
        # A carry refused with the object in the hand is planned again with a smaller margin.
        shares = config.carry_retry_margins or (1.0,)
        margin = config.carry_margin * shares[min(attempt.carry_failures, len(shares) - 1)]
        # A hold too weak for the speed is led no faster than the ticks left
        # require, and is not swung into its pose joint by joint.
        if attempt.fragile is None:
            attempt.fragile = bool(self._carry_step(None) < config.carry_fragile)
            attempt.plan_stats = dict(attempt.plan_stats or {}, fragile=attempt.fragile)
        haste = 1.0 if attempt.fragile else config.carry_haste
        if attempt.travel is None:
            path, stats = transport.plan_travel(scene, self._grids(scene), self.rng, self._hold, self._load(obs),
                                                config.episode_ticks - self.tick, config, margin=margin, haste=haste,
                                                fragile=attempt.fragile)
            if attempt.carry_failures:
                stats = dict(stats, margin_mm=round(1000 * margin, 1))
            rate = config.travel_rate
            if stats.get("way") == "joint by joint":
                rate = transport.hand_rate(scene, path, rate, config.travel_hand_step)
                stats = dict(stats, rate=round(rate, 4))
            limit = np.full(len(self._hold), rate)
            limit[robot.torso] = 0.002
            attempt.travel = PathStepper(path, limit)
            attempt.travel_wait = 0 if len(path) > 1 else config.travel_settle      # nothing to change: no wait
            attempt.plan_stats = dict(attempt.plan_stats or {}, travel=stats)
        if attempt.carry is None and (not attempt.travel.finished or attempt.travel_wait < config.travel_settle):
            if self._dropped(obs):
                return self._failed("object lost while taking the travel pose")
            if not attempt.travel.finished:
                return attempt.travel.step(), None
            attempt.travel_wait += 1
            return self._hold, None
        if attempt.carry is None:
            gap = scene.world.goal[:2] - np.asarray(obs["object_pos"], dtype=float)[:2]
            if np.linalg.norm(gap) < config.reach_near:
                return self._hold, State.REACH           # already there: no route to plan
            # A hold can be re-seated where the hand is level with the pads'
            # line level, and the pads hold the object about its middle: then
            # it sinks across the pads, and half a turn of the wrist undoes that.
            load = self._load(obs)
            attempt.seatable = bool(config.reseat_sag > 0 and transport.sag(self.env, robot, obs) is not None
                                    and load.lever < config.reseat_lever)
            attempt.carry, stats = transport.plan_carry(
                scene, self._grids(scene), self._hold, load, config.episode_ticks - self.tick, config, margin=margin,
                haste=haste)
            if isinstance(stats, dict):
                stats = dict(stats, lever_mm=round(1000 * load.lever), seatable=attempt.seatable)
            attempt.plan_stats = dict(attempt.plan_stats or {}, carry=stats)
            if attempt.carry is None:
                return self._failed("no clear route to the goal")
        if self._dropped(obs):
            attempt.plan_stats = dict(attempt.plan_stats or {}, lost=dict(
                tick=self.tick, phase=attempt.carry.phase, led_m=round(attempt.carry.led, 2)))
            return self._failed("object lost during the carry")
        if attempt.carry.reseats > attempt.reseats:
            # A re-seat has just ended. The drive rested through it: the pace
            # is set again from the ticks that are left, never under the plan's.
            attempt.reseats = attempt.carry.reseats
            ticks = max(config.episode_ticks - self.tick - config.reseat_slack, 1)
            attempt.carry.set_pace(haste * attempt.carry.left / ticks)
        if attempt.seatable and attempt.carry.phase == "drive":
            # An object sinking across the pads of a level hand is re-seated by half a turn of the wrist.
            sag = transport.sag(self.env, robot, obs)
            if sag is not None:
                attempt.sag_from = sag if attempt.sag_from is None else min(attempt.sag_from, sag)
                if sag > config.reseat_sag and sag - attempt.sag_from > 0.5 * config.reseat_sag and \
                        self.tick % 5 == 0 and self._roll_clear(scene, obs) and attempt.carry.reseat():
                    attempt.sag_from = None
                    attempt.plan_stats = dict(attempt.plan_stats, reseats=(attempt.plan_stats.get("reseats") or []) + [
                        dict(tick=self.tick, led_m=round(attempt.carry.led, 2), sag_mm=round(1000 * sag))])
        action, status = attempt.carry.step(robot.base_pose(obs["qpos"]))
        if status is Status.FAILED:
            return self._failed(attempt.carry.failure)
        return action, (State.REACH if status is Status.DONE else None)

    def _roll_clear(self, scene: Scene, obs) -> bool:
        """Whether the wrist can roll half a turn here, the object with it, clear of everything."""
        robot = scene.robot
        q = self._hold.copy()
        centre, orientation, _ = self._load(obs).at(q)
        scene.collision.focus(robot.base_pose(q)[:2])
        scene.collision.hold(q, centre, orientation)
        try:
            for angle in np.linspace(0.0, np.pi, 9)[1:]:
                rolled = q.copy()
                rolled[robot.arm[-1]] += angle
                if not scene.collision.is_free(rolled, self.config.reseat_room, load_margin=self.config.reseat_room):
                    return False
            return True
        finally:
            scene.collision.release()

    def _reach(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Put the object, not the hand, on the goal: first by
        moving the base and the torso with the arm still, then, for what they
        cannot do, by a straight move of the hand. Exit: DONE. Failures:
        object lost, goal out of reach."""
        scene, attempt, config = self._scene(obs), self.attempt, self.config
        distance = float(np.linalg.norm(scene.world.goal - np.asarray(obs["object_pos"], dtype=float)))
        if self._dropped(obs):
            return self._failed("object lost during the reach")
        if attempt.raising is not None:                  # the hand is bringing the object up to the goal's height
            if not attempt.raising.finished:
                return attempt.raising.step(), None
            attempt.raising = None
            return self._hold, None                      # the base and torso then close what is left
        if attempt.reach is None:
            if distance < 0.5 * config.goal_tolerance:
                self.report.delivered_tick = self.tick
                return self._hold, State.DONE
            if attempt.reach_wait < config.reach_approach_ticks:
                action, spent = transport.reach_step(scene, self._hold, self._load(obs), config)
                if action is not None and not spent:
                    attempt.reach_wait += 1
                    return action, None
            attempt.reach_wait = 0
            if distance < 0.8 * config.goal_tolerance:   # near enough already; the arm could only make it worse
                self.report.delivered_tick = self.tick
                return self._hold, State.DONE
            below = float(scene.world.goal[2] - obs["object_pos"][2])
            if not attempt.raised and below > 0.5 * config.goal_tolerance:
                attempt.raised = True
                path, stats = transport.plan_raise(scene, self.rng, self._hold, self._load(obs), config)
                attempt.plan_stats = dict(attempt.plan_stats or {}, raised=dict(stats, below_m=round(below, 3)))
                if path is not None:
                    limit = np.full(len(self._hold), config.travel_rate)
                    limit[scene.robot.torso] = 0.002
                    attempt.raising = PathStepper(path, limit)
                    return self._hold, None
            path = transport.plan_reach(scene, self.rng, self._hold, self._load(obs), config)
            if path is None:
                return self._failed("goal out of reach")
            limit = np.full(len(self._hold), 0.02)
            limit[scene.robot.torso] = 0.004
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
        """Segment 4. With the object still in the hand, go on carrying it:
        the carry and the reach are planned afresh, a set number of times.
        Otherwise start a fresh attempt from where the object now is (table
        or floor), if enough ticks remain. Exit: NAVIGATE, SETTLE or STOPPED."""
        scene, attempt = self._scene(obs), self.attempt
        if attempt.lifted_tick is not None and transport.in_hand(self.env, scene.robot, obs):
            attempt.carry, attempt.reach, attempt.reach_wait, attempt.lost = None, None, 0, 0
            attempt.raising = None
            attempt.carry_failures += 1
            attempt.plan_stats = dict(attempt.plan_stats or {}, carry_retries=attempt.carry_failures)
            gap = scene.world.goal[:2] - np.asarray(obs["object_pos"], dtype=float)[:2]
            if len(self.config.carry_retry_margins) > 1 and np.linalg.norm(gap) >= self.config.reach_near:
                attempt.travel, attempt.travel_wait = None, 0    # the pose is chosen again, with the smaller margin
            return self._hold, (State.NAVIGATE if attempt.carry_failures <= self.config.carry_retries
                                else State.STOPPED)
        if self.config.episode_ticks - self.tick < self.config.min_retry_ticks or \
                self.attempt.index + 1 >= self.config.max_attempts:
            return self._hold, State.STOPPED
        if attempt.away:
            # The object was dropped after the hand had left the grasp: open
            # the hand where it is. The way to the ready pose is planned from there.
            fingers = list(scene.robot.fingers)
            opened = self._hold.copy()
            opened[fingers] = scene.robot.upper[fingers]
            unwind = np.array([self._hold.copy(), opened])
        else:
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
