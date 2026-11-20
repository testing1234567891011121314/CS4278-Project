"""A collision-free arm motion that executes one grasp candidate (R8).

All geometry questions are answered in the planning simulator (R30, R31).
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.collision import CollisionChecker
from custom_libraries.common import Deadline
from custom_libraries.grasping import ik
from custom_libraries.grasping.candidates import GraspCandidate
from custom_libraries.planning_sim import PlanningSim
from custom_libraries.world import ObjectState


@dataclass(frozen=True)
class PlannerSettings:
    margin: float = 0.010               # least gap to the scene and the robot's own body in transit, metres
    close_margin: float = 0.002         # least gap to the scene during the approach and the lift
    object_margin: float = 0.0015       # least gap between the open hand and the object at the grasp
    tight_margin: float = -0.0015       # the same for a tight fit: the pads may press into the object this far
    transit_object_margin: float = 0.02  # least gap between the robot and the object in transit
    wedged: float = 0.003               # an object this deep inside a fixed body counts as wedged into it
    rim_offsets: tuple = (0.0, 0.004, -0.004, 0.008, -0.008)   # depths tried round a thin-wall grasp, metres
    rim_openings: tuple = (0.016, 0.012, 0.020, 0.024, 0.008)   # openings tried, beyond the wall's thickness
    rim_tip_inset: float = 0.004        # a thin wall is held at least this far from the fingertips
    pregrasp: float = 0.08              # the approach starts this far back along the approach axis
    lift: float = 0.30                  # longest straight lift planned, metres; the lift stops earlier, once the
                                        # object itself has risen enough (an object hanging in the hand rises less)
    least_lift: float = 0.12            # shortest lift accepted when the full one does not plan
    line_step: float = 0.01             # metres between waypoints of a straight move
    max_joint_jump: float = 0.30        # most a joint may change between such waypoints, rad
    ik_seconds: float = 2.0             # CPU-time guard for one grasp-pose solve. The solve is bounded by its
                                        # iteration counts; the guard only stops a runaway, so that results do
                                        # not depend on how busy the machine is (R24)
    ik_restarts: int = 4                # random restarts of a grasp-pose solve
    tree_step: float = 0.30             # how far one extension of a search tree goes, rad
    tree_seconds: float = 4.0           # CPU time for one transit search
    shortcuts: int = 40                 # attempts to straighten a found transit


@dataclass(frozen=True)
class GraspPlan:
    candidate: GraspCandidate
    transit: np.ndarray              # (K, 15) from where the arm was to the pregrasp pose, fingers open
    approach: np.ndarray             # (K, 15) straight in to the grasp pose
    drag: bool                       # grip only: the object is to be dragged, not lifted
    lift: np.ndarray                 # (K, 15) straight up from the grasp pose
    lift_height: float               # metres the lift path rises
    object_position: np.ndarray      # where the object was when this was planned
    object_orientation: np.ndarray
    grasp_in_object: tuple           # planned gripper frame in the object's frame: (position, 3x3 rotation)


def plan_grasp(candidates: list[GraspCandidate], qpos, obj: ObjectState, sim: PlanningSim,
               collision: CollisionChecker, rng: np.random.Generator, deadline: Deadline,
               settings: PlannerSettings = PlannerSettings(),
               drag_for=None) -> tuple[Optional[GraspPlan], dict]:
    """The first candidate, in order, for which the whole motion plans: grasp
    pose, straight approach, straight lift with the object in the hand, and a
    transit from `qpos`. Where `drag_for(candidate)` is true the plan ends
    with the hand in place: no lift is planned, the object is to be dragged.
    Returns the plan (or None) and what was tried."""
    stats = dict(tried=0, no_pose=0, pose_blocked=0, no_approach=0, no_lift=0, no_transit=0, chosen=None)
    robot = sim.robot
    start = np.array(qpos, dtype=float)
    sim.set_object(obj.position, obj.orientation)
    collision.release()
    collision.focus(robot.base_pose(start)[:2])
    for index, candidate in enumerate(candidates):
        if deadline.expired():
            break
        stats["tried"] += 1
        opened = start.copy()
        opened[list(robot.fingers)] = sim.gripper.finger_joint(candidate.opening)
        grasp = grasp_pose(candidate, opened, sim, collision, rng, deadline, settings, stats)
        if grasp is None:
            continue
        position, turn = candidate.gripper_pose()
        approach = straight_line(grasp, -candidate.approach_axis, settings.pregrasp, sim, collision, rng, deadline,
                                 settings, object_margin=hand_margin(candidate, settings))
        if approach is None or len(approach) < 2:
            stats["no_approach"] += 1
            continue
        approach = approach[::-1]                       # planned backwards from the grasp pose
        drag = bool(drag_for(candidate)) if drag_for is not None else False
        collision.hold(grasp, obj.position, obj.orientation)
        # An object wedged into the scene cannot be checked against it on the way out.
        wedged = collision.clearances(grasp, with_object=False)["load"] < -settings.wedged
        lift = np.array([grasp]) if drag else \
            straight_line(grasp, np.array([0.0, 0.0, 1.0]), settings.lift, sim, collision, rng, deadline,
                          settings, object_margin=0.0, load_after=None if wedged else 0.01,
                          least=settings.least_lift)
        collision.release()
        sim.set_object(obj.position, obj.orientation)
        if lift is None:
            stats["no_lift"] += 1
            continue
        transit = plan_transit(opened, approach[0], sim, collision, rng,
                               Deadline(min(settings.tree_seconds, deadline.remaining())), settings)
        if transit is None:
            stats["no_transit"] += 1
            continue
        stats["chosen"] = index
        lifted = sim.gripper_pose(lift[-1])[0][2] - sim.gripper_pose(grasp)[0][2]
        frame = _rotation(obj.orientation)
        return GraspPlan(candidate=candidate, transit=transit, approach=approach, drag=drag, lift=lift,
                         lift_height=float(lifted), object_position=obj.position.copy(),
                         object_orientation=obj.orientation.copy(),
                         grasp_in_object=(frame.T @ (position - obj.position), frame.T @ turn)), stats
    return None, stats


def hand_margin(candidate: GraspCandidate, settings: PlannerSettings) -> float:
    """Least gap between the open hand and the object for this kind of grasp."""
    return settings.tight_margin if candidate.kind == "tight" else settings.object_margin


def hands_that_fit(candidates: list[GraspCandidate], obj: ObjectState, sim: PlanningSim,
                   collision: CollisionChecker, settings: PlannerSettings = PlannerSettings()) -> list[GraspCandidate]:
    """The candidates at which the open hand, on its own, touches neither the
    object nor the scene, in their order. This costs no arm solve, so it is
    run on every candidate before any is tested for reach."""
    sim.set_object(obj.position, obj.orientation)
    collision.focus(obj.centre[:2])
    roomy, tight = [], []
    near = collision.near

    def gaps(candidate: GraspCandidate) -> Optional[float]:
        """The hand's gap to the object if the hand fits at this candidate, else None."""
        position, turn = candidate.gripper_pose()
        to_object, to_scene = sim.hand_clearance(position, turn, candidate.opening, near)
        fits = to_object >= hand_margin(candidate, settings) and to_scene >= settings.close_margin
        return to_object if fits else None

    for candidate in candidates:
        gap = gaps(candidate)
        if gap is None and candidate.kind == "rim":
            # A thin wall's real thickness and edge are not in the cloud: try
            # the same grasp a little wider and a little deeper or shallower.
            last = sim.gripper.pad_tip - settings.rim_tip_inset
            for offset in (candidate.offset + d for d in settings.rim_offsets):
                for extra in settings.rim_openings:
                    opening = min(sim.gripper.max_opening, candidate.width + extra)
                    if not 0.0 <= offset <= last or (offset == candidate.offset and opening == candidate.opening):
                        continue
                    variant = dataclasses.replace(candidate, offset=float(offset), opening=float(opening))
                    gap = gaps(variant)
                    if gap is not None:
                        candidate = variant
                        break
                if gap is not None:
                    break
        if gap is not None:
            (tight if candidate.kind == "tight" else roomy).append((gap, candidate))
    # A tight fit goes on most easily where the real shape is narrowest: the
    # cloud cannot tell, the planning copy can.
    tight.sort(key=lambda pair: -pair[0])
    return [c for _, c in roomy] + [c for _, c in tight]


def grasp_pose(candidate: GraspCandidate, opened, sim: PlanningSim, collision: CollisionChecker,
               rng: np.random.Generator, deadline: Deadline, settings: PlannerSettings,
               stats: Optional[dict] = None) -> Optional[np.ndarray]:
    """Joint values that put the open hand on the candidate without touching
    anything, or None. The hand is symmetric, so both ways round are tried."""
    position, turn = candidate.gripper_pose()
    blocked = False
    for flip in (np.eye(3), np.diag([1.0, -1.0, -1.0])):
        budget = Deadline(min(settings.ik_seconds, deadline.remaining()))
        q = ik.solve(sim, position, turn @ flip, opened, rng, budget, restarts=settings.ik_restarts)
        if q is None:
            continue
        if collision.is_free(q, settings.close_margin, object_margin=hand_margin(candidate, settings)):
            return q
        blocked = True
    if stats is not None:
        stats["pose_blocked" if blocked else "no_pose"] += 1
    return None


def straight_line(q_start, direction, distance: float, sim: PlanningSim, collision: CollisionChecker,
                  rng: np.random.Generator, deadline: Deadline, settings: PlannerSettings,
                  object_margin: Optional[float] = None, load_after: Optional[float] = None,
                  least: Optional[float] = None) -> Optional[np.ndarray]:
    """A (K, 15) joint path that moves the gripper `distance` metres along
    `direction` in a straight line with its orientation fixed, starting at
    `q_start`. Each waypoint continues from the one before, so the arm does
    not flip between solutions. With `load_after`, the held object is checked
    against the scene once the gripper has moved that far (before that it
    still touches what it rested on). With `least`, a shorter line of at least
    that length is accepted. None if the line cannot be followed."""
    direction = np.asarray(direction, dtype=float)
    origin, turn = sim.gripper_pose(q_start)
    joints = list(sim.robot.plan_joints)
    path = [np.array(q_start, dtype=float)]
    steps = int(round(distance / settings.line_step))
    for k in range(1, steps + 1):
        travelled = k * settings.line_step
        q = ik.solve(sim, origin + direction * travelled, turn, path[-1], rng, deadline, restarts=0)
        loaded = None if load_after is None or travelled <= load_after else 0.0
        ok = q is not None and np.abs(q[joints] - path[-1][joints]).max() <= settings.max_joint_jump and \
            collision.is_free(q, settings.close_margin, object_margin=object_margin, load_margin=loaded)
        if not ok:
            reached = (k - 1) * settings.line_step
            return np.array(path) if least is not None and reached >= least else None
        path.append(q)
    return np.array(path)


def plan_transit(q_start, q_goal, sim: PlanningSim, collision: CollisionChecker, rng: np.random.Generator,
                 deadline: Deadline, settings: PlannerSettings = PlannerSettings(),
                 avoid_object: bool = True) -> Optional[np.ndarray]:
    """A collision-free (K, 15) joint path from `q_start` to `q_goal` that
    moves only the torso and arm. The straight joint-space move is tried
    first; otherwise two random trees are grown toward each other from both
    ends and the path found is straightened. None if none is found in time.
    With `avoid_object` off the object is not treated as an obstacle (used to
    get the arm away after a failed grasp, when the object may lie against it)."""
    near = settings.transit_object_margin if avoid_object else None
    touch = 0.0 if avoid_object else None
    robot = sim.robot
    joints = list(robot.plan_joints)
    a, b = np.array(q_start, dtype=float), np.array(q_goal, dtype=float)
    b[joints] = np.where(robot.wraps[joints], a[joints] + (b[joints] - a[joints] + np.pi) % (2 * np.pi) - np.pi,
                         b[joints])

    def free(q) -> bool:
        return collision.is_free(q, settings.margin, object_margin=near)

    def edge_free(p0, p1) -> bool:
        return collision.path_is_free(np.array([p0, p1]), settings.margin, object_margin=near, step=0.06)

    # The ends themselves may sit closer to things than the transit margin
    # allows (the pregrasp pose is 8 cm from the object).
    if not collision.is_free(a, 0.0, object_margin=touch) or not collision.is_free(b, 0.0, object_margin=touch):
        return None
    if _ends_free(a, b, edge_free, collision, settings, touch):
        return np.array([a, b])

    lower = np.where(robot.wraps[joints], a[joints] - np.pi, robot.lower[joints])
    upper = np.where(robot.wraps[joints], a[joints] + np.pi, robot.upper[joints])
    trees = [[(a, -1)], [(b, -1)]]                      # each node: (configuration, index of its parent)

    def extend(tree, toward) -> Optional[int]:
        """Grow `tree` from its node nearest `toward` by one step; the new node's index, or None."""
        gaps = [np.linalg.norm(node[0][joints] - toward[joints]) for node in tree]
        near = int(np.argmin(gaps))
        if gaps[near] < 1e-9:
            return None
        step = min(1.0, settings.tree_step / gaps[near])
        new = tree[near][0] + step * (toward - tree[near][0])
        if not free(new) or not edge_free(tree[near][0], new):
            return None
        tree.append((new, near))
        return len(tree) - 1

    def trace(tree, index) -> list:
        out = []
        while index >= 0:
            out.append(tree[index][0])
            index = tree[index][1]
        return out

    grow = 0
    while not deadline.expired():
        sample = a.copy()
        sample[joints] = rng.uniform(lower, upper)
        new = extend(trees[grow], sample)
        if new is not None:
            target = trees[grow][new][0]
            other = trees[1 - grow]
            while True:                                  # pull the other tree toward the new node
                reached = extend(other, target)
                if reached is None:
                    break
                if np.linalg.norm(other[reached][0][joints] - target[joints]) < 1e-9:
                    half_a = trace(trees[0], new if grow == 0 else reached)[::-1]
                    half_b = trace(trees[1], reached if grow == 0 else new)
                    path = half_a + half_b[1:]
                    return _straighten(np.array(path), edge_free, rng, settings.shortcuts)
        grow = 1 - grow
    return None


def _ends_free(a, b, edge_free, collision: CollisionChecker, settings: PlannerSettings, touch) -> bool:
    """True if the straight joint move from `a` to `b` is free, allowing the
    last stretch to come as close to things as its end pose does."""
    return edge_free(a, b) or (edge_free(a, a + 0.8 * (b - a)) and collision.path_is_free(
        np.array([a + 0.8 * (b - a), b]), settings.close_margin,
        object_margin=None if touch is None else settings.close_margin, step=0.03))


def _straighten(path: np.ndarray, edge_free, rng: np.random.Generator, attempts: int) -> np.ndarray:
    """Replace stretches of the path by straight moves wherever those are free."""
    path = list(path)
    for _ in range(attempts):
        if len(path) < 3:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j - i >= 2 and edge_free(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    return np.array(path)


def _rotation(quaternion) -> np.ndarray:
    from custom_libraries.planning_sim import rotation
    return rotation(quaternion)
