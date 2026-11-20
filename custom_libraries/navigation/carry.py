"""Carrying the held object from the table to the maze exit (R14, R16, R17).

The arm keeps the pose it lifted the object in. The object is led along the
route; the base follows on the same route one arm's length behind, heading
toward the object, so the arm stays on the inside of every corner. Where the
hand points straight down, the wrist's roll keeps the pads' closing line
along the object's direction of travel: the base's speed jumps at every tick,
and along that line a pad pushes the object, where across it only friction
holds it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.common import Status
from custom_libraries.navigation import a_star
from custom_libraries.navigation.grid import OccupancyGrid
from custom_libraries.robot import RobotModel, wrap_angle


@dataclass(frozen=True)
class CarrySettings:
    fillet: float = 0.5             # corners of the route are rounded over up to this length of each leg, metres
    max_step: float = 0.030         # fastest the object is led, metres per tick
    min_step: float = 0.004         # slowest, however heavy the object or weak the grip
    ramp: float = 0.0015            # change of the object's step per tick
    max_yaw: float = 0.05           # most the base turns per tick, rad
    max_roll: float = 0.04          # most the wrist rolls per tick, rad: faster rolls slide objects out
    torso_step: float = 0.002       # the torso rises this far per tick, metres, before the turn. Every tick's
                                    # rise jolts the object upward: at 4 mm per tick the apple was lost later
                                    # in the carry. Rising 0.3 mm per tick all through the drive instead was
                                    # tried and cost the strawberry and the hammer (NAVIGATE.md, attempt 10)
    upright: float = 0.9            # the hand counts as pointing down if its axis is this close to vertical
    max_lag: float = 0.20           # the drive fails if the base falls this far behind, metres
    tolerance: float = 0.015        # the base counts as arrived within this distance
    settle_ticks: int = 40


def half_turn(angle):
    """An angle between lines, brought into [-pi/2, pi/2): a line is the same after half a turn."""
    return (np.asarray(angle, dtype=float) + np.pi / 2.0) % np.pi - np.pi / 2.0


def round_corners(path: np.ndarray, reach: float, points: int = 6) -> np.ndarray:
    """The polyline with each corner replaced by a curve that leaves the
    corner's legs at most `reach` metres, and at most half a leg, from it."""
    path = np.asarray(path, dtype=float)
    if len(path) < 3:
        return path
    out = [path[0]]
    for a, b, c in zip(path[:-2], path[1:-1], path[2:]):
        before, after = np.linalg.norm(b - a), np.linalg.norm(c - b)
        cut = min(reach, before / 2.0, after / 2.0)
        if cut < 1e-6:
            out.append(b)
            continue
        start, end = b + (a - b) * (cut / before), b + (c - b) * (cut / after)
        for t in np.linspace(0.0, 1.0, points):          # a parabola through the two cut points, pulled to the corner
            out.append((1 - t) ** 2 * start + 2 * (1 - t) * t * b + t ** 2 * end)
    out.append(path[-1])
    return np.array(out)


@dataclass(frozen=True)
class CarryPlan:
    path: np.ndarray            # (K, 2) world route: starts at the base, ends with the object on the goal
    arm: float                  # horizontal distance from the base centre to the object
    bearing: float              # direction of the object as seen from the base, relative to the base's heading
    yaw_now: float              # the base's heading when the plan was made
    turn: float                 # the turn on the spot that points the arm along the route's first leg, rad
    roll_sign: int              # how the closing line turns with the wrist roll: +1, -1, or 0 if it cannot be steered
    closing: float              # heading of the closing line minus the base's heading, at the roll below
    roll_now: float             # the wrist roll when the plan was made
    step: float                 # fastest the object is led, metres per tick

    @property
    def marks(self) -> np.ndarray:
        return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))])

    @property
    def length(self) -> float:
        return float(self.marks[-1])

    def point(self, along: float) -> np.ndarray:
        marks = self.marks
        return np.array([np.interp(along, marks, self.path[:, 0]), np.interp(along, marks, self.path[:, 1])])

    def pose(self, along: float) -> tuple[np.ndarray, float]:
        """The base's (x, y, yaw) and the heading of the object's travel when
        the object is `along` metres down the route (at least one arm)."""
        lead = self.point(along)
        low, high = max(0.0, along - 2.0 * self.arm), max(0.0, along - self.arm)
        # The base is the point of the route one arm behind the object as the
        # crow flies; round a corner that is farther back along the route.
        if np.linalg.norm(lead - self.point(low)) <= self.arm:
            back = low
        else:
            for _ in range(30):
                mid = (low + high) / 2.0
                if np.linalg.norm(lead - self.point(mid)) > self.arm:
                    low = mid
                else:
                    high = mid
            back = (low + high) / 2.0
        base = self.point(back)
        toward = lead - base
        ahead = self.point(min(self.length, along + 0.02)) - self.point(max(0.0, along - 0.02))
        heading = float(np.arctan2(ahead[1], ahead[0]))
        return np.array([base[0], base[1], float(np.arctan2(toward[1], toward[0])) - self.bearing]), heading

    def roll(self, yaw: float, heading: float, near: float) -> float:
        """The wrist roll that lays the closing line along `heading` when the
        base heads `yaw`: the solution nearest the roll `near`."""
        if self.roll_sign == 0:
            return near
        wanted = self.roll_now + self.roll_sign * float(half_turn(heading - (yaw + self.closing)))
        return wanted + np.pi * round((near - wanted) / np.pi)


def plan_route(grids: list[OccupancyGrid], start, goal, settings: CarrySettings) -> list[np.ndarray]:
    """Routes for the base from `start` to `goal`, one per grid that has one
    (the grids differ in how far they keep from the walls), each shortened
    and with its corners rounded."""
    routes = []
    for grid in grids:
        path = a_star.find_path(grid, start, goal)
        if path is not None:
            routes.append(round_corners(a_star.shorten(path, grid), settings.fillet))
    return routes


def make_plan(route: np.ndarray, base_pose, object_xy, closing_heading: Optional[float], roll_now: float,
              roll_sign: int, step: float) -> CarryPlan:
    """A plan that leads the object along `route`. `closing_heading` is the
    world heading of the pads' closing line now, or None if the hand does not
    point down and the line cannot be steered by the wrist's roll."""
    base_pose = np.asarray(base_pose, dtype=float)
    toward = np.asarray(object_xy, dtype=float)[:2] - base_pose[:2]
    bearing = float(wrap_angle(np.arctan2(toward[1], toward[0]) - base_pose[2]))
    first = route[1] - route[0]
    start_yaw = float(np.arctan2(first[1], first[0])) - bearing
    steer = closing_heading is not None and roll_sign != 0
    return CarryPlan(path=np.asarray(route, dtype=float), arm=float(np.linalg.norm(toward)), bearing=bearing,
                     yaw_now=float(base_pose[2]), turn=float(wrap_angle(start_yaw - base_pose[2])),
                     roll_sign=roll_sign if steer else 0,
                     closing=float(closing_heading - base_pose[2]) if steer else 0.0,
                     roll_now=float(roll_now), step=float(step))


def poses_to_check(plan: CarryPlan, turn: float, spacing: float = 0.05) -> list[tuple[np.ndarray, float]]:
    """Base poses and wrist rolls over the whole carry (the turn on the spot
    by `turn`, then the drive), close enough together to check for collisions."""
    out = []
    base = plan.path[0]
    roll = plan.roll(plan.yaw_now, plan.yaw_now + plan.bearing + np.pi / 2.0, plan.roll_now)
    for t in np.linspace(0.0, 1.0, max(2, int(abs(turn) / 0.05) + 1)):
        out.append((np.array([base[0], base[1], plan.yaw_now + t * turn]), roll))
    for along in np.arange(plan.arm, plan.length + 1e-9, spacing):
        pose, heading = plan.pose(along)
        roll = plan.roll(pose[2], heading, roll)
        out.append((pose, roll))
    return out


class CarryFollower:
    """Executes a carry plan tick by tick: raise the torso so the object rides
    at the goal's height; roll the closing line across the arm; turn on the
    spot until the arm points down the route; roll the closing line along the
    route; drive. Every rate is limited, and the object's speed is ramped."""

    def __init__(self, plan: CarryPlan, robot: RobotModel, hold: np.ndarray, rise: float, turn: float,
                 settings: CarrySettings = CarrySettings()):
        """`hold` is the (15,) action the arm is to keep; `rise` is how far the
        torso is to go up; `turn` is the turn on the spot (the plan's, or the
        long way round if the short way is blocked)."""
        self.plan, self.robot, self.settings = plan, robot, settings
        self._target = np.array(hold, dtype=float)
        self._roll_joint = robot.arm[-1]
        self._torso_goal = float(self._target[robot.torso] + rise)
        self._yaw = plan.yaw_now
        self._yaw_goal = plan.yaw_now + turn
        self._along, self._step, self._turn_rate = plan.arm, 0.0, 0.0
        self._waited = 0
        self.phase = "raise"
        self.failure: Optional[str] = None

    def _roll_toward(self, wanted: float) -> bool:
        """Move the roll target toward `wanted`; True once it is there."""
        gap = wanted - self._target[self._roll_joint]
        self._target[self._roll_joint] += float(np.clip(gap, -self.settings.max_roll, self.settings.max_roll))
        return abs(gap) <= self.settings.max_roll

    def step(self, base_pose) -> tuple[np.ndarray, Status]:
        """The next (15,) action, given the base's measured world pose. DONE
        when the base rests at the end of the route. FAILED if it cannot keep up."""
        s, plan, robot = self.settings, self.plan, self.robot
        base_pose = np.asarray(base_pose, dtype=float)
        start = plan.path[0]

        if self.phase == "raise":
            gap = self._torso_goal - self._target[robot.torso]
            self._target[robot.torso] += float(np.clip(gap, -s.torso_step, s.torso_step))
            if abs(gap) <= s.torso_step:
                self.phase = "roll across"
        elif self.phase == "roll across":
            # While the base turns on the spot the object moves across the arm.
            across = self._yaw + plan.bearing + np.pi / 2.0
            if abs(self._yaw_goal - self._yaw) < 1e-6 or \
                    self._roll_toward(plan.roll(self._yaw, across, self._target[self._roll_joint])):
                self.phase = "turn"
        elif self.phase == "turn":
            left = self._yaw_goal - self._yaw
            limit = min(s.max_yaw, plan.step / max(plan.arm, 1e-6))      # the object goes round at arm's length
            stopping = np.sqrt(2.0 * (s.ramp / max(plan.arm, 1e-6)) * abs(left))
            self._turn_rate = min(self._turn_rate + s.ramp / max(plan.arm, 1e-6), limit, max(stopping, 1e-4))
            self._yaw += float(np.clip(left, -self._turn_rate, self._turn_rate))
            if abs(self._yaw_goal - self._yaw) < 1e-9:
                self.phase = "roll along"
        elif self.phase == "roll along":
            _, heading = plan.pose(plan.arm)
            if self._roll_toward(plan.roll(self._yaw, heading, self._target[self._roll_joint])):
                self.phase = "drive"
        elif self.phase == "drive":
            left = plan.length - self._along
            stopping = np.sqrt(2.0 * s.ramp * max(left, 0.0))
            step = min(self._step + s.ramp, plan.step, max(stopping, s.ramp))
            for _ in range(8):                           # shorten the step until every rate is within its limit
                along = min(plan.length, self._along + step)
                pose, heading = plan.pose(along)
                roll = plan.roll(pose[2], heading, self._target[self._roll_joint])
                turning = abs(float(wrap_angle(pose[2] - self._yaw)))
                rolling = abs(roll - self._target[self._roll_joint])
                if turning <= s.max_yaw and rolling <= s.max_roll:
                    break
                step *= 0.6
            self._along, self._step = along, step
            self._yaw += float(wrap_angle(pose[2] - self._yaw))
            self._target[self._roll_joint] += float(np.clip(roll - self._target[self._roll_joint],
                                                            -s.max_roll, s.max_roll))
            self._target = robot.with_base(self._target, [pose[0], pose[1], self._yaw])
            lag = float(np.linalg.norm(pose[:2] - base_pose[:2]))
            if lag > s.max_lag:
                self.failure = "base fell behind its route"
                return self._target.copy(), Status.FAILED
            if self._along >= plan.length:
                if lag < s.tolerance:
                    return self._target.copy(), Status.DONE
                self._waited += 1
                if self._waited > s.settle_ticks:
                    self.failure = "base did not come to rest at the end of its route"
                    return self._target.copy(), Status.FAILED
            return self._target.copy(), Status.RUNNING

        self._target = robot.with_base(self._target, [start[0], start[1], self._yaw])
        return self._target.copy(), Status.RUNNING
