"""Where the base stands to pick the object, and how it gets there."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.collision import CollisionChecker
from custom_libraries.common import Deadline
from custom_libraries.grasping.arm_planner import PlannerSettings, grasp_pose
from custom_libraries.grasping.candidates import GraspCandidate
from custom_libraries.navigation import a_star
from custom_libraries.navigation.base_follower import BaseFollower
from custom_libraries.navigation.grid import OccupancyGrid
from custom_libraries.planning_sim import PlanningSim
from custom_libraries.world import ObjectState


@dataclass(frozen=True)
class ParkingSettings:
    distances: tuple = (0.70, 0.60, 0.80, 0.50, 0.90)   # metres from the object to the base centre, in order tried
    directions: int = 16                # places tried around the object at each distance
    margin: float = 0.02                # least gap between the robot and the scene while driving, metres
    max_poses: int = 14                 # parking poses examined in full before giving up
    max_candidates: int = 12            # grasp candidates tested for reach at each pose
    back_off: float = 0.6               # farthest the drive's last, straight stretch starts from the pose
    exclude_radius: float = 0.15        # a pose this close to one already tried counts as the same
    max_step: float = 0.07              # base speed on the way to the object, metres per tick
    max_turn: float = 0.10              # radians per tick
    ramp: float = 0.007                 # change of the step per tick
    travel_weight: float = 0.2          # preference, in score units, for grasps that close along the line from
                                        # the base to the object: the base backs off along that line, and a push
                                        # along the pads' normal does not slide the object across them
    reach_restarts: int = 2             # random restarts of the solve that tests whether a candidate is in reach
    line_restarts: int = 12             # the same when the base must stand on a given line (raking)


@dataclass(frozen=True)
class ParkingPose:
    x: float
    y: float
    yaw: float
    path: np.ndarray                    # (K, 2) world route from where the base was to here
    reachable: tuple[int, ...]          # indices of the candidates within reach from here, best first

    @property
    def pose(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw])


def make_follower(path: np.ndarray, start_yaw: float, final_yaw: float, settings: ParkingSettings) -> BaseFollower:
    return BaseFollower(path, start_yaw, final_yaw, settings.max_step, settings.max_turn, settings.ramp)


def choose_parking(candidates: list[GraspCandidate], obj: ObjectState, grid: OccupancyGrid, sim: PlanningSim,
                   collision: CollisionChecker, qpos, rng: np.random.Generator, deadline: Deadline,
                   exclude: tuple = (), settings: ParkingSettings = ParkingSettings(),
                   planner: PlannerSettings = PlannerSettings(), facing=None, tiers=None) -> tuple[Optional[ParkingPose], dict]:
    """A pose for the base, facing the object, such that: the robot (with the
    arm as it is in `qpos`) touches nothing there or on the way; a route leads
    to it; and at least one grasp candidate can be reached from it. Places on
    the side the base is already on are tried first. `exclude` lists (x, y)
    of poses already tried in this trial. `facing` may give a horizontal
    direction from the object: only places on that line are then tried.
    Returns the pose (None if there is
    none) and a count of why places were turned down."""
    robot = sim.robot
    here = robot.base_pose(qpos)
    target = obj.centre[:2]
    away = here[:2] - target
    offsets = sorted(np.arange(settings.directions) * 2.0 * np.pi / settings.directions,
                     key=lambda a: abs((a + np.pi) % (2.0 * np.pi) - np.pi))
    if facing is not None:
        facing_angle, offsets = float(np.arctan2(facing[1], facing[0])), [0.0]
    else:
        facing_angle = float(np.arctan2(away[1], away[0]))
    sim.set_object(obj.position, obj.orientation)
    collision.release()
    stats = dict(blocked=0, no_route=0, drive_blocked=0, out_of_reach=0, examined=0, out_of_time=False)
    # A place on a given line has few alternatives, so each is tried harder.
    planner = dataclasses.replace(
        planner, ik_restarts=settings.reach_restarts if facing is None else settings.line_restarts)
    for offset in offsets:
        for distance in settings.distances:
            if deadline.expired() or stats["examined"] >= settings.max_poses:
                stats["out_of_time"] = deadline.expired()
                return None, stats
            angle = facing_angle + offset
            outward = np.array([np.cos(angle), np.sin(angle)])
            spot = target + distance * outward
            if any(np.linalg.norm(spot - np.asarray(e)[:2]) < settings.exclude_radius for e in exclude):
                continue
            pose = np.array([spot[0], spot[1], angle + np.pi])       # facing the object
            collision.focus(spot)
            parked = robot.with_base(qpos, pose)
            if not collision.is_free(parked, settings.margin, object_margin=settings.margin):
                stats["blocked"] += 1
                continue
            path = _route(grid, here[:2], spot, outward, settings)
            if path is None:
                stats["no_route"] += 1
                continue
            follower = make_follower(path, here[2], pose[2], settings)
            if not all(collision.is_free(robot.with_base(qpos, p), settings.margin, object_margin=settings.margin)
                       for p in follower.poses()):
                stats["drive_blocked"] += 1
                continue
            stats["examined"] += 1
            reachable = []
            for index in travel_order(candidates, outward, settings.travel_weight, tiers)[:settings.max_candidates]:
                candidate = candidates[index]
                opened = parked.copy()
                opened[list(robot.fingers)] = sim.gripper.finger_joint(candidate.opening)
                if grasp_pose(candidate, opened, sim, collision, rng, deadline, planner) is not None:
                    reachable.append(index)
                    if len(reachable) >= 3:
                        break
            if reachable:
                stats["distance"] = float(distance)
                return ParkingPose(float(spot[0]), float(spot[1]), float(pose[2]), path, tuple(reachable)), stats
            stats["out_of_reach"] += 1
    return None, stats


def travel_order(candidates: list[GraspCandidate], direction, weight: float, tiers=None) -> list[int]:
    """Indices of the candidates, best first, where a grasp that closes along
    the horizontal `direction` counts for up to `weight` more. `tiers` gives
    each candidate a rank that comes before its score: every candidate of a
    lower tier is tried before any of a higher one."""
    direction = np.asarray(direction, dtype=float)[:2]
    direction = direction / max(np.linalg.norm(direction), 1e-9)
    worth = [c.score + weight * abs(float(c.closing_axis[:2] @ direction)) for c in candidates]
    tiers = [0] * len(candidates) if tiers is None else tiers
    return sorted(range(len(candidates)), key=lambda k: (tiers[k], -worth[k], k))


def _route(grid: OccupancyGrid, start, spot, outward, settings: ParkingSettings) -> Optional[np.ndarray]:
    """A route on the grid to the nearest free point straight out from the
    parking spot, then straight in. None if there is none."""
    for back in np.arange(0.0, settings.back_off + 1e-9, grid.resolution):
        staging = spot + back * outward
        if not grid.is_free(staging):
            continue
        path = a_star.find_path(grid, start, staging)
        if path is None:
            return None
        path = a_star.shorten(path, grid)
        return path if back == 0.0 else np.vstack([path, spot])
    return None
