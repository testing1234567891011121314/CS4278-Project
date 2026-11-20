"""Pick-Nav-Reach solver: one state machine, one action per tick.

Drop-in for the course's KeyBoardController (R1):

    solver = Solver(env)
    while True:
        obs, info = env.step(solver.get_action())

The state machine's spine (dispatch, tick budgets, the exception guard, the
retry bookkeeping) is here. What each state does is a stub; see docs/plan.md
for the segment each belongs to.
"""
from __future__ import annotations

import dataclasses
import traceback
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from custom_libraries.grasping.arm_planner import GraspPlan
from custom_libraries.grasping.candidates import GraspCandidate
from custom_libraries.navigation.carry import CarryPlan
from custom_libraries.navigation.parking import ParkingPose


class State(Enum):
    SETTLE = "settle"                # segment 1: wait for the object to come to rest
    GO_TO_OBJECT = "go_to_object"    # segment 1: choose a parking pose and drive to it
    GRASP = "grasp"                  # segment 2: plan, close, verify, lift, hold
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

    # Tick budget of each state (R19). Starting values sized from the reference
    # solver's phase lengths; to be re-measured in segment 0.
    settle_ticks: int = 150
    go_to_object_ticks: int = 250
    grasp_ticks: int = 200
    navigate_ticks: int = 2000
    reach_ticks: int = 200

    seed: Optional[int] = None       # R24; None draws fresh entropy
    strict: bool = False             # tests: let exceptions out of get_action

    def budget(self, state: State) -> int:
        """Ticks the solver may spend in `state` before it counts as a failure."""
        return {
            State.SETTLE: self.settle_ticks,
            State.GO_TO_OBJECT: self.go_to_object_ticks,
            State.GRASP: self.grasp_ticks,
            State.NAVIGATE: self.navigate_ticks,
            State.REACH: self.reach_ticks,
        }.get(state, self.episode_ticks)


@dataclass
class Attempt:
    """Everything one pick attempt knows. A retry gets a new one (R21)."""

    index: int
    started_tick: int
    pick_mode: Optional[str] = None                 # "table" or "floor"
    candidates: list[GraspCandidate] = field(default_factory=list)
    candidate: Optional[GraspCandidate] = None      # the one being executed
    parking: Optional[ParkingPose] = None
    grasp_plan: Optional[GraspPlan] = None
    carry_plan: Optional[CarryPlan] = None
    lifted_tick: Optional[int] = None
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
        self._state_entered = 0
        self._hold = self._measured_pose()       # last commanded action, latched (R23)
        self._handlers = {
            State.SETTLE: self._settle,
            State.GO_TO_OBJECT: self._go_to_object,
            State.GRASP: self._grasp,
            State.NAVIGATE: self._navigate,
            State.REACH: self._reach,
            State.RETRY: self._retry,
        }

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

    def _new_attempt(self) -> None:
        """Close the current attempt in the report and start a fresh one (R21)."""
        a = self.attempt
        self.report.attempts.append(dict(index=a.index, started_tick=a.started_tick, pick_mode=a.pick_mode,
                                         lifted_tick=a.lifted_tick, failure=a.failure))
        self.attempt = Attempt(index=a.index + 1, started_tick=self.tick)

    def _measured_pose(self) -> np.ndarray:
        try:
            return np.asarray(self.env._get_obs()["qpos"], dtype=float).copy()
        except Exception:
            if self.config.strict:
                raise
            return np.zeros(int(getattr(self.env, "action_size", 15)))

    # ----------------------------------------------------------------- states
    # Each handler takes the observation and returns (action, next state or
    # None to stay). A failure is `self._fail(reason)` plus the held action.

    def _settle(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 1. Wait until the object has stopped moving, then note
        whether it rests on the table or the floor.
        Exit: GO_TO_OBJECT. Budget spent: go on with the object as it is."""
        raise NotImplementedError("segment 1: settle")

    def _go_to_object(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 1. Generate grasp candidates, choose a parking pose, plan
        and drive the base path, face the object.
        Exit: GRASP. Failures: no candidates, no parking pose, no route,
        base blocked."""
        raise NotImplementedError("segment 1: go to object")

    def _grasp(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 2. Plan the arm motion for the best candidate that plans,
        execute it, close, verify the hold, lift, hold 50 ticks.
        Exit: NAVIGATE. Failures: no plan, object moved, hold not verified,
        object lost."""
        raise NotImplementedError("segment 2: grasp")

    def _navigate(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Back away from the table, plan the carry, drive the
        route to the exit.
        Exit: REACH. Failures: object lost, no route, base blocked."""
        raise NotImplementedError("segment 3: navigate")

    def _reach(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 3. Move the arm so the object is at the goal.
        Exit: DONE. Failures: object lost, goal out of reach."""
        raise NotImplementedError("segment 3: reach")

    def _retry(self, obs) -> tuple[np.ndarray, Optional[State]]:
        """Segment 4. Decide whether enough ticks remain for another attempt;
        if so start one from where the object now is.
        Exit: SETTLE with a fresh attempt, or STOPPED."""
        raise NotImplementedError("segment 4: retry")
