"""Travel poses: how the hand and arm are posed for the carry, with the grasp
left as the pick made it (docs/plan.md, "Segment 3 from here").

A travel pose says which way the hand points and how the pads' line lies; the
arm is folded so the hand sits on the base's centre line. Where the pads sit
on the object is not part of it: that is the grasp, and the grasp is not
changed for the carry's sake.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.common import Budget
from custom_libraries.grasping import ik
from custom_libraries.robot import wrap_angle


def solve_arm(sim, position, turn: np.ndarray, seed_qpos, budget: Budget, position_tolerance: float = 0.001,
              angle_tolerance: float = 0.005, iterations: int = 80, damping: float = 0.05) -> Optional[np.ndarray]:
    """Joint values, within limits, that put the gripper frame at the world
    `position` with the 3x3 rotation `turn`, found from `seed_qpos`. Only the
    torso and arm move. None if not found.

    The same iteration as the pick's `grasping.ik.solve`, without restarts
    and with one difference: a joint already on a limit that the step would
    push past it is left out of that step, and the others make the move. The
    pick's solver stops there. From an arm stretched down to the floor it
    could not turn the hand at all, so an object picked from the floor could
    not be raised to the goal (runs/deliver_try06). The pick's solver is not
    changed."""
    robot = sim.robot
    joints = list(robot.plan_joints)
    limited = ~robot.wraps[joints]
    lower, upper = robot.lower[joints], robot.upper[joints]
    seed = np.array(seed_qpos, dtype=float)
    target = np.asarray(position, dtype=float)
    q = seed.copy()
    for _ in range(iterations):
        if budget.expired():
            return None
        budget.spend()
        here, facing = sim.gripper_pose(q)
        error = np.concatenate([target - here, ik.rotation_vector(turn @ facing.T)])
        if np.linalg.norm(error[:3]) < position_tolerance and np.linalg.norm(error[3:]) < angle_tolerance:
            q[joints] = np.where(limited, q[joints], seed[joints] + wrap_angle(q[joints] - seed[joints]))
            return q
        error[:3] *= min(1.0, 0.10 / max(np.linalg.norm(error[:3]), 1e-12))     # bound the step
        error[3:] *= min(1.0, 0.50 / max(np.linalg.norm(error[3:]), 1e-12))
        j = ik.jacobian(sim, q)
        step = j.T @ np.linalg.solve(j @ j.T + damping ** 2 * np.eye(6), error)
        stuck = limited & (((q[joints] >= upper - 1e-9) & (step > 0)) | ((q[joints] <= lower + 1e-9) & (step < 0)))
        if stuck.any():
            free = j[:, ~stuck]
            step = np.zeros(len(joints))
            step[~stuck] = free.T @ np.linalg.solve(free @ free.T + damping ** 2 * np.eye(6), error)
        q[joints] = np.where(limited, np.clip(q[joints] + step, lower, upper), q[joints] + step)
    return None


def axis_rotation(axis, angle: float) -> np.ndarray:
    """The 3x3 rotation by `angle` radians about the unit vector `axis`."""
    x, y, z = np.asarray(axis, dtype=float)
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


@dataclass(frozen=True)
class TravelPose:
    name: str
    elevation: Optional[float]      # of the hand's axis above the horizontal, degrees (-90 points straight down);
                                    # None keeps the hand as it lifted the object (turned to point forward)
    pads: str = "level"             # the pads' line: "level"; "upright" (one finger under the object); or, with
                                    # the hand kept as it lifted the object, "forward" (the line along the
                                    # base's heading, so the base's yaw alone keeps the pads pushing along the
                                    # travel) or "sideways" (across it)
    fold: bool = True               # False: the arm stays exactly as it lifted the object
    steer: str = "along"            # the wrist's roll during the drive, for a hand pointing down: keeps the
                                    # pads' line "along" or "across" the travel, or "none" (the roll is left alone)
    reach: float = 0.70             # the object is brought in to this far from the base's centre, metres


# The base turns with the route in every one of these: its heading follows
# the travel, which is what keeps the gripper lined up with it.
SURVEY = (
    TravelPose("as lifted", None, fold=False),
    TravelPose("folded 0.70", None),
    TravelPose("folded 0.40", None, reach=0.40),
    TravelPose("pads forward 0.70", None, pads="forward", steer="none"),
    TravelPose("pads forward 0.40", None, pads="forward", steer="none", reach=0.40),
    TravelPose("pads sideways 0.40", None, pads="sideways", steer="none", reach=0.40),
    TravelPose("down, pads along", -90.0, steer="along"),
    TravelPose("down, pads across", -90.0, steer="across"),
    TravelPose("level", 0.0),
    TravelPose("up 45", 45.0),
    # One finger under the object: its weight rests on that finger instead of
    # hanging on the pads' friction.
    TravelPose("level, finger under", 0.0, pads="upright"),
    TravelPose("level, finger under, 0.40", 0.0, pads="upright", reach=0.40),
    TravelPose("up 30, finger under", 30.0, pads="upright"),     # the weight also settles the object toward the palm
)


def hand_rotation(pose: TravelPose, turn: np.ndarray, forward: np.ndarray) -> np.ndarray:
    """The 3x3 rotation that takes the hand's present orientation `turn`
    (columns: its axis, the pads' line, across both) to the pose, for a base
    heading along the unit vector `forward`."""
    up = np.array([0.0, 0.0, 1.0])
    approach, closing = turn[:, 0], turn[:, 1]
    if pose.elevation is None and pose.pads in ("forward", "sideways"):
        # Turn the hand about the vertical, the shorter way, until the pads'
        # line lies along the base's heading (or across it).
        line = closing[:2]
        if np.linalg.norm(line) < 0.3:                  # the pads' line is close to vertical: nothing to line up
            return np.eye(3)
        wanted = forward if pose.pads == "forward" else np.cross(up, forward)
        turn_by = float(np.arctan2(wanted[1], wanted[0]) - np.arctan2(line[1], line[0]))
        turn_by = (turn_by + np.pi / 2.0) % np.pi - np.pi / 2.0      # a line is the same after half a turn
        return axis_rotation(up, turn_by)
    if pose.elevation is None:
        if np.linalg.norm(approach[:2]) > 0.3:          # not pointing straight down: turn it to point forward
            return axis_rotation(up, float(np.arctan2(forward[1], forward[0]) -
                                              np.arctan2(approach[1], approach[0])))
        return np.eye(3)
    if pose.elevation <= -89.0:
        tilt = np.cross(approach, -up)                  # the smallest turn that stands the hand upright
        if np.linalg.norm(tilt) < 1e-6:
            return np.eye(3)
        return axis_rotation(tilt / np.linalg.norm(tilt), float(np.arcsin(min(1.0, np.linalg.norm(tilt)))))
    e = np.radians(pose.elevation)
    left = np.cross(up, forward)
    axis = np.cos(e) * forward + np.sin(e) * up
    line = left if pose.pads == "level" else np.cross(axis, left)
    if line @ closing < 0.0:                            # the hand is the same after half a turn about its axis
        line = -line
    target = np.column_stack([axis, line, np.cross(axis, line)])
    return target @ turn.T


def fold_path(scene, rng: np.random.Generator, q0: np.ndarray, grip: np.ndarray, centre: np.ndarray,
              orientation, spin: np.ndarray, reach: float, work: int = 12000,
              rigid: bool = True, height: Optional[float] = None) -> tuple[np.ndarray, dict]:
    """A (K, 15) joint path from `q0` that turns the hand by `spin` about the
    point `grip` (where the pads hold the object) while bringing that point
    onto the base's centre line, `reach` metres ahead of the base's centre at
    most, with the arm folded into the vertical plane through that line.
    With `height` the object's centre is also brought to that height above
    the floor; without it the grip point stays at the height it has.

    `centre` and `orientation` are the object's pose now; with `rigid` it is
    taken to move with the hand and is checked against the scene and the
    robot. Returns the path (one row if no such pose is within reach and
    clear) and a record of what was found."""
    robot, sim = scene.robot, scene.sim
    q0 = np.array(q0, dtype=float)
    position, turn = sim.gripper_pose(q0)
    base = robot.base_pose(q0)
    forward = np.array([np.cos(base[2]), np.sin(base[2]), 0.0])
    left = np.array([-forward[1], forward[0], 0.0])
    up = np.array([0.0, 0.0, 1.0])
    carried = spin @ (np.asarray(centre, dtype=float) - grip)
    reach = min(float(np.linalg.norm(np.asarray(centre)[:2] - base[:2])), reach)
    grip_to = np.array([base[0], base[1], grip[2]]) + reach * forward - [carried[0], carried[1], 0.0]
    if height is not None:
        grip_to[2] = height - carried[2]
    turn_to = spin @ turn
    position_to = grip_to + spin @ (position - grip)
    stats: dict = {}
    joints = list(robot.plan_joints)

    scene.collision.focus(base[:2])
    scene.collision.hold(q0, centre, orientation)
    try:
        def clear(q) -> bool:
            # The env does not collide the robot with itself, so the arm may
            # pass close to the body; the object may not.
            gaps = scene.collision.clearances(q, with_object=rigid)
            return gaps["scene"] >= 0.01 and gaps["own"] >= 0.0 and \
                (not rigid or (gaps["load"] >= 0.005 and gaps["object"] >= 0.005))

        def spread(q) -> float:
            """How far the arm sticks out to the side of the base's centre line, metres."""
            sim.set_robot(q)
            return max(abs(float((sim.link_pose(robot.joint_indices[k])[0] - [base[0], base[1], 0.0]) @ left))
                       for k in robot.arm)

        # The end pose, from seeds with the arm in the plane of the centre
        # line, the elbow above it or below it: a level hand found only from
        # the first seeds tried had its elbow 17 to 24 cm to the side, which
        # does not pass this maze (runs/deliver_13).
        # Every seed gets its full 80 iterations; `work` is then allowed
        # to each end pose's way in, separately. (One allowance for all of it
        # ran out on the first end pose that failed late, and the rest were
        # reported "out of reach at step 1"; runs/deliver_try05.)
        lifts, elbows, flexes = (-1.1, -0.8, -0.2, 0.4), (-2.0, -1.2, 0.8, 1.6, 2.1), (-1.4, -0.8, 0.4, 1.4)
        budget, found = Budget(len(lifts) * len(elbows) * len(flexes) * 80), []
        pan, lift, upper_roll, elbow, fore_roll, flex, _ = robot.arm
        for seed_lift in lifts:
            for seed_elbow in elbows:
                for seed_flex in flexes:
                    seed = q0.copy()
                    seed[[pan, upper_roll, fore_roll]] = 0.0
                    seed[[lift, elbow, flex]] = seed_lift, seed_elbow, seed_flex
                    q = solve_arm(sim, position_to, turn_to, seed, budget)
                    if q is not None and clear(q):
                        found.append(q)
        if not found:
            stats["kept"] = "no folded pose within reach and clear"
            return np.array([q0]), stats
        stats["end_spreads_cm"] = sorted({round(100 * spread(q)) for q in found})
        vector = ik.rotation_vector(spin)
        angle = float(np.linalg.norm(vector))
        axis = vector / angle if angle > 1e-9 else up

        def leg(start, end, turned: tuple, moved: tuple, name: str) -> Optional[list]:
            """Joint values from `start` to `end` along which the hand turns
            from `turned[0]` to `turned[1]` of the whole turn and moves from
            `moved[0]` to `moved[1]` of the whole way, each step continuing
            from the one before. None if the arm cannot follow or is blocked."""
            swing = angle * abs(turned[1] - turned[0])
            count = max(2, int(np.ceil(max(np.abs(end[joints] - start[joints]).max() / 0.05, swing / 0.05))))
            out, last = [], np.array(start, dtype=float)
            for k in range(1, count + 1):
                t = k / count
                part = axis_rotation(axis, (turned[0] + t * (turned[1] - turned[0])) * angle)
                shift = (moved[0] + t * (moved[1] - moved[0])) * (grip_to - grip)
                q = solve_arm(sim, grip + shift + part @ (position - grip), part @ turn,
                              last + (end - start) / count, budget)
                if q is None or np.abs(q[joints] - last[joints]).max() > 0.3:
                    stats["blocked"] = f"{name}, step {k} of {count}: " + \
                        ("out of work" if q is None and budget.expired() else
                         "out of reach" if q is None else "the arm would jump")
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
            """The turn and the move at once; failing that, the turn where the hand is, then the move."""
            both = leg(q0, q1, (0.0, 1.0), (0.0, 1.0), "turning while folding")
            if both is not None:
                return [q0] + both
            turned = leg(q0, q0, (0.0, 1.0), (0.0, 0.0), "turning in place")
            if turned is None:
                return None
            folded = leg(turned[-1], q1, (1.0, 1.0), (0.0, 1.0), "folding after the turn")
            return None if folded is None else [q0] + turned + folded

        for q1 in sorted(found, key=spread)[:6]:        # the narrowest end pose whose way in is clear
            budget = Budget(work)
            q1 = q1.copy()
            q1[joints] = np.where(robot.wraps[joints], q0[joints] + wrap_angle(q1[joints] - q0[joints]), q1[joints])
            path = way(q1)
            if path is not None:
                stats.update(spread_cm=[round(100 * spread(q0)), round(100 * spread(q1))], reach_m=round(reach, 2))
                return np.array(path), stats
        stats["kept"] = "the way into every folded pose is blocked"
        return np.array([q0]), stats
    finally:
        scene.collision.release()

