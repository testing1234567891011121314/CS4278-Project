"""Everything after the pick: where to back off to, the travel pose, the carry
and the final reach (docs/plan.md, "Segment 3 from here").

The grasp is left exactly as the pick made it. What is chosen here is how the
hand and arm are posed for the drive (`plan_travel_pose`), the route and its
speed (`plan_carry`), and where the base goes first so that both are possible
(`choose_back_off`). Everything is planned in the planning copy; the env's
simulator is only read, for the contacts between the hand and the object.

`config` is the solver's Config: the settings used here are its `carry_*`,
`travel_*`, `back_off_*` and `reach_*` fields.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pybullet as p

from custom_libraries.common import Budget
from custom_libraries.grasping import arm_planner
from custom_libraries.navigation import carry
from custom_libraries.navigation.base_follower import BaseFollower
from custom_libraries.navigation.grid import OccupancyGrid
from custom_libraries.navigation.travel_pose import TravelPose, fold_path, hand_rotation
from custom_libraries.planning_sim import rotation

# The poses the surveys of 2026-11-21 left standing (runs/travel_01 to travel_03, NAVIGATE_V2.md).
DOWN = TravelPose("down, pads along", -90.0, steer="along")     # the hand stood straight down, the wrist steering
FOLDED = TravelPose("folded", None)                             # the hand as it lifted the object, turned forward
LEVEL = TravelPose("level", 0.0)                                # the hand level and forward, the pads' line level
UNDER = TravelPose("level, finger under", 0.0, pads="upright")  # the same with one finger under the object


def grip_point(env, robot) -> tuple[Optional[np.ndarray], float]:
    """Where the pads hold the object (the middle of the fingers' contacts,
    weighted by force) and the sum of those forces. (None, 0) without contact."""
    points = [(np.array(c[6]), c[9]) for c in
              p.getContactPoints(env.robot_id, env.object_id, physicsClientId=env.pb_physics_client)
              if c[3] in robot.finger_links and c[9] > 0]
    weight = sum(force for _, force in points)
    return (sum(point * force for point, force in points) / weight, weight) if weight > 0 else (None, 0.0)


def in_hand(env, robot, obs, reach: float = 0.25) -> bool:
    """True while the object is still in the hand: it presses on a finger or
    on the palm, and its centre is within `reach` metres of the hand. Weaker
    than the pick's `is_held` (both pads) on purpose: an object that has
    shifted onto one pad and the palm is still being carried."""
    links = {robot.gripper_link, *robot.finger_links}
    touching = any(c[3] in links and c[9] > 0.5 for c in
                   p.getContactPoints(env.robot_id, env.object_id, physicsClientId=env.pb_physics_client))
    if not touching:
        return False
    hand = np.array(p.getLinkState(env.robot_id, robot.gripper_link, computeForwardKinematics=True,
                                   physicsClientId=env.pb_physics_client)[4])
    return bool(np.linalg.norm(np.asarray(obs["object_pos"], dtype=float) - hand) < reach)


def nearest_link(scene, q) -> str:
    """The name of the robot link nearest a fixed body with the robot at `q`,
    and the gap in millimetres: for the record of a refused carry, not for checks."""
    sim = scene.sim
    sim.set_robot(q)
    gap, link = scene.collision.look, None
    for body in scene.collision.near:
        for c in p.getClosestPoints(sim.robot_id, body, scene.collision.look, physicsClientId=sim.client):
            if body == sim.floor_id and c[3] not in scene.collision.arm_links:
                continue
            if c[8] < gap:
                gap, link = c[8], c[3]
    if link is None:
        return "none"
    name = "base" if link == -1 else p.getJointInfo(sim.robot_id, link, physicsClientId=sim.client)[12].decode()
    return f"{name} {1000 * gap:.0f} mm"


def carry_grids(scene, config) -> list[OccupancyGrid]:
    """Floor maps for the carry's route, one per distance kept from the walls."""
    return [OccupancyGrid.from_world(scene.world, config.grid_resolution, keep, obstacle_radius=keep)
            for keep in config.carry_clearances]


class Load:
    """The held object as the planner carries it: fixed in the hand."""

    def __init__(self, scene, q, position, orientation, grip: Optional[np.ndarray], mass: float = 0.0):
        sim = scene.sim
        hand, turn = sim.gripper_pose(q)
        self._scene = scene
        self.mass = float(mass)                         # kg
        self._centre = turn.T @ (np.asarray(position, dtype=float) - hand)
        self._turn = turn.T @ rotation(orientation)
        if grip is None:                                # no contact read: the middle of the pads
            grip = hand + turn[:, 0] * (sim.gripper.pad_back + sim.gripper.pad_tip) / 2.0
        self._grip = turn.T @ (np.asarray(grip, dtype=float) - hand)

    def at(self, q) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The object's centre, its orientation (xyzw) and the grip point with the robot at `q`."""
        from custom_libraries.collision import _quaternion
        hand, turn = self._scene.sim.gripper_pose(q)
        return hand + turn @ self._centre, _quaternion(turn @ self._turn), hand + turn @ self._grip


def travel_poses(turn: np.ndarray, mass: float, config) -> list[TravelPose]:
    """The poses to try, in order, for a hand whose orientation is `turn`
    holding `mass` kg. From the two surveys of the whole route at 15 mm per
    tick (runs/travel_02, runs/travel_03; one trial per object and pose):

    - A hand that does not point mostly down (the plate's, the master chef
      can's) is kept as it is and turned forward. Stood down, the can jammed
      against the palm and the plate was out of reach.
    - A heavy object in a hand that points down slides out along the fingers
      (the sugar box 5 mm per metre). With the hand level it slides the other
      way, into the palm, and stops there: level first.
    - Otherwise one finger under the object first: that carried the most
      objects the whole route, and the hammer, apple and bowl in both
      surveys. Then the hand stood straight down, the wrist keeping the pads'
      line along the travel, which is all the mug and the marker can take.
    No pose carried every object; NAVIGATE_V2.md has the table."""
    if turn[2, 0] > -config.travel_upright:
        return [FOLDED]
    return ([LEVEL] if mass >= config.travel_heavy else []) + [UNDER, DOWN, FOLDED]


def plan_travel_pose(scene, rng: np.random.Generator, q, load: Load, config) -> tuple[np.ndarray, dict]:
    """The (K, 15) move from `q` into the travel pose: the first pose of
    `travel_poses` that is within reach with a clear way in, with the object
    at the goal's height if that is within reach too (the final reach is then
    only a few centimetres). One row (the arm stays as it lifted the object)
    if none is."""
    robot, sim = scene.robot, scene.sim
    q = np.array(q, dtype=float)
    _, turn = sim.gripper_pose(q)
    base = robot.base_pose(q)
    forward = np.array([np.cos(base[2]), np.sin(base[2]), 0.0])
    centre, orientation, grip = load.at(q)
    tried = {}
    for pose in travel_poses(turn, load.mass, config):
        for height in (float(scene.world.goal[2]), None):
            path, stats = fold_path(scene, rng, q, grip, centre, orientation, hand_rotation(pose, turn, forward),
                                    config.travel_reach, work=config.travel_work, height=height)
            if len(path) >= 2:
                return path, dict(stats, pose=pose.name, at_goal_height=height is not None, not_reached=tried)
            tried[pose.name + (" at the goal's height" if height is not None else "")] = \
                stats.get("blocked", stats.get("kept"))
    return np.array([q]), dict(pose="as lifted", not_reached=tried)


def plan_travel(scene, grids: list[OccupancyGrid], rng: np.random.Generator, q, load: Load, ticks_left: int,
                config) -> tuple[np.ndarray, dict]:
    """The (K, 15) move from `q` into the pose the object will travel in: the
    first pose of `travel_poses` that is within reach, has a clear way in,
    and in which the whole carry to the goal is clear. A pose is not taken
    just because the arm can reach it: with the hand level the elbow stands
    17 to 24 cm to the side, and on this maze's route that touched a wall in
    10 of 23 carries (runs/travel_03) and was refused outright in six of
    fifteen trials (runs/deliver_13).

    If no pose passes the carry's check, the first within reach is taken and
    the carry is planned from it as it comes. One row if none is within reach."""
    robot, sim = scene.robot, scene.sim
    q = np.array(q, dtype=float)
    _, turn = sim.gripper_pose(q)
    base = robot.base_pose(q)
    forward = np.array([np.cos(base[2]), np.sin(base[2]), 0.0])
    centre, orientation, grip = load.at(q)
    tried, fallback = {}, None
    for pose in travel_poses(turn, load.mass, config):
        for height in (float(scene.world.goal[2]), None):
            name = pose.name + (" at the goal's height" if height is not None else "")
            path, stats = fold_path(scene, rng, q, grip, centre, orientation, hand_rotation(pose, turn, forward),
                                    config.travel_reach, work=config.travel_work, height=height)
            if len(path) < 2:
                tried[name] = stats.get("blocked", stats.get("kept"))
                continue
            record = dict(stats, pose=pose.name, at_goal_height=height is not None)
            follower, why = plan_carry(scene, grids, path[-1], load, ticks_left, config)
            if follower is not None:
                return path, dict(record, not_reached=tried)
            nearest = why.get("blocked") or [{}]
            tried[name] = f"carry blocked: {nearest[0].get('nearest')} at base {nearest[0].get('base')}, " \
                          f"arm spread {stats.get('spread_cm')} cm of {stats.get('end_spreads_cm')}"
            fallback = fallback or (path, record)
            break                                       # the same pose at another height is no narrower
    if fallback is not None:
        return fallback[0], dict(fallback[1], not_reached=tried, carry_blocked=True)
    return np.array([q]), dict(pose="as lifted", not_reached=tried)


def plan_carry(scene, grids: list[OccupancyGrid], q, load: Load, ticks_left: int, config,
               whole_route: bool = True) -> tuple[Optional[carry.CarryFollower], dict]:
    """Choose a route to the goal and check the whole carry along it, the
    object in the hand, against the scene. `q` is the pose the arm keeps.
    With `whole_route` False only the turn on the spot and the first metres
    are checked (enough to tell whether a carry can start from here).
    Returns the follower, or None and why."""
    robot, world = scene.robot, scene.world
    settings = carry.CarrySettings(turn_step=config.carry_turn_step)
    q = np.array(q, dtype=float)
    base = robot.base_pose(q)
    _, hand = scene.sim.gripper_pose(q)
    position, orientation, _ = load.at(q)
    upright = abs(hand[2, 0]) >= settings.upright         # the hand's axis is close to vertical
    closing = float(np.arctan2(hand[1, 1], hand[0, 1])) if upright else None
    # The torso takes the object toward the goal's height; the reach does the rest.
    room = (robot.lower[robot.torso] + 0.005 - q[robot.torso], robot.upper[robot.torso] - 0.005 - q[robot.torso])
    rise = float(np.clip(world.goal[2] - position[2], min(room[0], 0.0), max(room[1], 0.0)))
    raised = q.copy()
    raised[robot.torso] += rise
    roll_joint = robot.arm[-1]
    scene.collision.hold(q, position, orientation)
    blocked = []
    # Floor maps on which the base stands clear come first: on the others the route begins with a hook.
    grids = sorted(grids, key=lambda grid: not grid.is_free(base[:2]))
    try:
        for route in carry.plan_route(grids, base[:2], world.goal[:2], settings):
            # Slower is safer for the hold, so the object is led only a
            # little faster than the ticks that are left require.
            arm = float(np.linalg.norm(position[:2] - base[:2]))
            length = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
            # An object that will arrive well under the goal has to be raised there: ticks are kept back for it.
            low = world.goal[2] - position[2] - rise > 0.5 * config.goal_tolerance
            needed = (length - arm) / max(ticks_left - config.carry_reserve - (config.raise_reserve if low else 0), 1)
            step = float(np.clip(config.carry_haste * needed, settings.min_step, settings.max_step))
            plan = carry.make_plan(route, base, position[:2], closing, q[roll_joint], int(np.sign(hand[2, 0])), step)
            if plan.length <= plan.arm:
                continue
            other = plan.turn - np.sign(plan.turn) * 2.0 * np.pi if abs(plan.turn) > 1e-6 else plan.turn
            for turn in dict.fromkeys([plan.turn, other]):          # the short way round, then the long way
                poses = carry.poses_to_check(plan, turn)
                if not whole_route:
                    poses = poses[:max(2, int(abs(turn) / 0.05) + 1) + int(config.back_off_look / 0.05)]
                clear = True
                for index, (pose, roll) in enumerate(poses):
                    at = robot.with_base(raised, pose)
                    at[roll_joint] = roll
                    scene.collision.focus(pose[:2])
                    if not scene.collision.is_free(at, config.carry_margin, load_margin=config.carry_margin):
                        gaps = scene.collision.clearances(at, with_object=False)
                        blocked.append(dict(pose=index, turn_deg=round(float(np.degrees(turn))),
                                            base=[round(float(v), 2) for v in pose],
                                            nearest=nearest_link(scene, at),
                                            gaps_mm={k: round(1000 * v) for k, v in gaps.items()}))
                        clear = False
                        break
                if clear:
                    stats = dict(route_m=round(plan.length, 2), step_mm=round(1000 * step, 1), steered=bool(upright),
                                 rise_m=round(rise, 3), turn_deg=round(float(np.degrees(turn))))
                    return carry.CarryFollower(plan, robot, q, rise, turn, settings), stats
        return None, dict(blocked=blocked[:4])
    finally:
        scene.collision.release()


def choose_back_off(scene, grids: list[OccupancyGrid], rng: np.random.Generator, q, load: Load, away,
                    step: float, ramp: float, ticks_left: int, config) -> tuple[Optional[BaseFollower], dict]:
    """Where the base goes with the object before the travel pose is taken.

    Chosen from the scene, not fixed: the shortest straight retreat (none at
    all, if that will do) along which the robot and the object are clear and
    from whose end a route can begin, the travel pose can be taken and the
    carry can start. Straight `away` (the direction from where the object
    stood to the base) is tried at every length first, then the other
    headings, spread round from it. If no retreat leads to a place the carry
    can start from, the longest clear one straight away is taken, as the pick
    did before, and the carry is planned from there as it comes.

    "A route can begin" means the base stands clear of the walls and the
    table by the margin the route itself keeps. From nearer, the route starts
    with a hook back out of that margin, and the object is swung round the
    hook at the speed of the drive (the bleach bottle, runs/travel_trace03).

    Returns the follower (None: stay here) and a record of what was found."""
    robot = scene.robot
    q = np.array(q, dtype=float)
    base = robot.base_pose(q)
    away = np.asarray(away, dtype=float)[:2]
    bearing = float(np.arctan2(away[1], away[0])) if np.linalg.norm(away) > 1e-6 else base[2] + np.pi
    options = [(0.0, 0.0)] + [(distance, 0.0) for distance in config.back_off_distances] + \
        [(distance, np.radians(turn)) for distance in config.back_off_distances
         for turn in config.back_off_headings if turn != 0.0]
    stats = dict(tried=[])
    plain = None                                         # the longest clear retreat straight away

    def follower_for(distance: float, turn: float) -> Optional[BaseFollower]:
        outward = np.array([np.cos(bearing + turn), np.sin(bearing + turn)])
        follower = BaseFollower(np.array([base[:2], base[:2] + distance * outward]), base[2], None, step, 0.03, ramp)
        centre, orientation, _ = load.at(q)
        scene.collision.focus(base[:2] + 0.5 * distance * outward)
        scene.collision.hold(q, centre, orientation)
        try:
            clear = all(scene.collision.is_free(robot.with_base(q, pose), 0.01, load_margin=0.005)
                        for pose in follower.poses())
        finally:
            scene.collision.release()
        return follower if clear else None

    def carry_starts_from(end) -> Optional[str]:
        """None if the travel pose can be taken and the carry started with the base at `end`; else why not."""
        if not any(grid.is_free(end[:2]) for grid in grids):
            return "no room for a route to begin"
        there = robot.with_base(q, end)
        path, pose = plan_travel_pose(scene, rng, there, load, config)
        if len(path) < 2:
            return "no travel pose"
        follower, _ = plan_carry(scene, grids, path[-1], load, ticks_left, config, whole_route=False)
        return None if follower is not None else "carry blocked"

    for distance, turn in options[:config.back_off_tries]:
        follower = follower_for(distance, turn) if distance > 0.0 else None
        if distance > 0.0 and follower is None:
            stats["tried"].append([distance, round(float(np.degrees(turn))), "way blocked"])
            continue
        why = carry_starts_from(base if follower is None else follower.pose_at(1.0))
        stats["tried"].append([distance, round(float(np.degrees(turn))), why or "chosen"])
        if why is None:
            stats.update(distance_m=distance, heading_deg=round(float(np.degrees(turn))))
            return follower, stats
    for distance in config.back_off:                     # nothing better found: as far straight away as is clear
        plain = follower_for(distance, 0.0)
        if plain is not None:
            stats.update(distance_m=distance, heading_deg=0, fallback=True)
            return plain, stats
    stats.update(distance_m=0.0, fallback=True)
    return None, stats


def reach_step(scene, q, load: Load, config) -> tuple[Optional[np.ndarray], bool]:
    """One tick of the final approach with the arm still: the base moves the
    object toward the goal on the floor plan and the torso moves it toward
    the goal's height, a little each. Returns the (15,) action, or None if
    that move is blocked, and whether the base and torso can do no more
    (what is left is then for the arm, `plan_reach`)."""
    robot = scene.robot
    q = np.array(q, dtype=float)
    position, orientation, _ = load.at(q)
    gap = scene.world.goal - position
    base = robot.base_pose(q)
    shift = 0.5 * gap[:2]
    far = float(np.linalg.norm(shift))
    if far > config.reach_step:
        shift *= config.reach_step / far
    out = robot.with_base(q, [base[0] + shift[0], base[1] + shift[1], base[2]])
    torso = robot.torso
    out[torso] = float(np.clip(q[torso] + np.clip(0.5 * gap[2], -0.002, 0.002),
                               robot.lower[torso] + 0.005, robot.upper[torso] - 0.005))
    spent = float(np.linalg.norm(gap[:2])) < 0.01 and (abs(gap[2]) < 0.01 or abs(out[torso] - q[torso]) < 1e-6)
    scene.collision.focus(base[:2])
    scene.collision.hold(q, position, orientation)
    try:
        free = scene.collision.is_free(out, 0.005, load_margin=0.005)
    finally:
        scene.collision.release()
    return (out if free else None), spent


def plan_raise(scene, rng: np.random.Generator, q, load: Load, config) -> tuple[Optional[np.ndarray], dict]:
    """The (K, 15) move, with the base standing at the goal, that brings the
    object up to the goal's height where it is. An object picked from the
    floor travels low, and a hand pointing down cannot lift it 0.9 m (the
    wrist would have to go above the arm's reach: the strawberry and the
    apple stopped 55 cm under the goal, runs/deliver_try04). Laying the hand
    level brings the object up from the middle of the arm's range instead;
    here the robot is standing still, so what the level poses let go of
    during a drive does not matter. None if no pose gets it there."""
    robot, sim = scene.robot, scene.sim
    q = np.array(q, dtype=float)
    _, turn = sim.gripper_pose(q)
    base = robot.base_pose(q)
    forward = np.array([np.cos(base[2]), np.sin(base[2]), 0.0])
    centre, orientation, grip = load.at(q)
    tried = {}
    for pose in (LEVEL, UNDER, TravelPose("up 45", 45.0), DOWN, FOLDED):
        path, stats = fold_path(scene, rng, q, grip, centre, orientation, hand_rotation(pose, turn, forward),
                                config.travel_reach, work=config.travel_work, height=float(scene.world.goal[2]))
        if len(path) >= 2:
            return path, dict(stats, pose=pose.name, not_reached=tried)
        tried[pose.name] = stats.get("blocked", stats.get("kept"))
    return None, dict(not_reached=tried)


def plan_reach(scene, rng: np.random.Generator, q, load: Load, config) -> Optional[np.ndarray]:
    """The (K, 15) straight move of the hand that puts the object, not the
    hand, on the goal. None if the goal is out of reach."""
    robot = scene.robot
    position, orientation, _ = load.at(q)
    gap = scene.world.goal - position
    distance = float(np.linalg.norm(gap))
    scene.collision.hold(q, position, orientation)
    scene.collision.focus(robot.base_pose(q)[:2])
    try:
        path = arm_planner.straight_line(q, gap / distance, distance, scene.sim, scene.collision, rng,
                                         Budget(config.reach_work), arm_planner.PlannerSettings(), load_after=0.0,
                                         least=max(0.01, distance - 0.5 * config.goal_tolerance))
    finally:
        scene.collision.release()
    return path if path is not None and len(path) >= 2 else None
