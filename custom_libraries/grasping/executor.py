"""Follows a grasp plan tick by tick and checks the hold (R9 to R12, R22).

The env's simulator is only read here: contact points between the hand and
the object tell whether the object is held (R30).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import pybullet as p

from custom_libraries.common import Status
from custom_libraries.grasping.arm_planner import GraspPlan
from custom_libraries.planning_sim import PlanningSim, rotation
from custom_libraries.robot import RobotModel


@dataclass(frozen=True)
class ExecutorSettings:
    transit_step: float = 0.06          # most an arm joint's target moves per tick in transit, rad. Where the
                                        # wedged marker lands when the hand's approach throws it out depends on
                                        # this (see GRASP.md): 0.06 open floor, 0.09 and 0.12 under the table
    approach_step: float = 0.04         # ... during the straight approach
    lift_step: float = 0.02             # ... during the lift, with the object in the hand
    torso_scale: float = 0.2            # the torso's target moves this many metres per radian of the above
    arrive_tolerance: float = 0.02      # joint error at which a path's end counts as reached, rad
    arrive_ticks: int = 25              # ticks allowed for the arm to settle at a path's end
    moved_tolerance: float = 0.015      # the object may shift this far before the close, metres
    tight_step: float = 0.015           # approach speed for a tight fit, rad per tick: the pads rub the object
    close_step: float = 0.003           # each finger's target closes this far per tick, metres
    squeeze: float = 0.030              # the finger targets go this far past the contact, metres
    contact_force: float = 1.0          # least normal force that counts as a contact, newtons
    grip_ticks: int = 8                 # ticks both fingers must hold the object before the lift
    lost_ticks: int = 4                 # ticks without the hold that count as losing the object
    lift_goal: float = 0.15             # the lift ends once the object has risen this far, metres


class PathStepper:
    """Moves a joint target along a (K, 15) path, at most `limit` per joint per tick."""

    def __init__(self, path: np.ndarray, limit: np.ndarray):
        self.path = np.asarray(path, dtype=float)
        self._limit = limit
        self._next = 1
        self.target = self.path[0].copy()

    @property
    def finished(self) -> bool:
        return self._next >= len(self.path)

    def step(self) -> np.ndarray:
        budget = 1.0                                     # the share of one tick's motion still unused
        while not self.finished and budget > 1e-9:
            gap = self.path[self._next] - self.target
            need = float(np.max(np.abs(gap) / self._limit))
            if need <= budget:
                self.target = self.path[self._next].copy()
                self._next += 1
                budget -= need
            else:
                self.target = self.target + gap * (budget / need)
                budget = 0.0
        return self.target.copy()


def finger_contacts(env, robot: RobotModel, least_force: float) -> tuple[bool, bool]:
    """Whether each finger presses on the object, read from the env's simulator."""
    points = p.getContactPoints(env.robot_id, env.object_id, physicsClientId=env.pb_physics_client)
    pressed = {c[3] for c in points if c[9] > least_force}
    return robot.finger_links[0] in pressed, robot.finger_links[1] in pressed


def is_held(env, robot: RobotModel, least_force: float = 1.0) -> bool:
    """True while both pads press on the object."""
    return all(finger_contacts(env, robot, least_force))


def grasp_error(env, robot: RobotModel, plan: GraspPlan, obs) -> tuple[float, float]:
    """Metres and radians between the planned and the actual gripper frame,
    both relative to the object (R12)."""
    state = p.getLinkState(env.robot_id, robot.gripper_link, computeForwardKinematics=True,
                           physicsClientId=env.pb_physics_client)
    frame = rotation(obs["object_xyzw"])
    position = frame.T @ (np.array(state[4]) - np.asarray(obs["object_pos"], dtype=float))
    turn = frame.T @ rotation(state[5])
    planned_position, planned_turn = plan.grasp_in_object
    # The hand is symmetric: half a turn about its own axis is the same grasp.
    angles = []
    for flip in (np.eye(3), np.diag([1.0, -1.0, -1.0])):
        cos = (np.trace(planned_turn @ flip @ turn.T) - 1.0) / 2.0
        angles.append(float(np.arccos(np.clip(cos, -1.0, 1.0))))
    return float(np.linalg.norm(position - planned_position)), min(angles)


class GraspExecutor:
    """One per attempt. `step` is called once per tick until it stops running.

    Phases, in order: transit, approach, close, grip, lift, hold. A rake ends
    after the approach, with the hand in place ("gripped")."""

    def __init__(self, plan: GraspPlan, env, sim: PlanningSim, lift_height: float, hold_ticks: int,
                 settings: ExecutorSettings = ExecutorSettings()):
        self.plan, self.env, self.settings = plan, env, settings
        self.robot, self.gripper = sim.robot, sim.gripper
        self._lift_height, self._hold_ticks = lift_height, hold_ticks
        self.phase = "transit"
        self.failure: Optional[str] = None
        self.lifted_tick_count: Optional[int] = None      # ticks from the start of the lift until it was high enough
        self._stepper = self._stepper_for(plan.transit, settings.transit_step)
        self._target = plan.transit[0].copy()
        self._waited = 0
        self._counter = 0
        self._lost = 0
        self._rest_height = float(plan.object_position[2])

    def _stepper_for(self, path: np.ndarray, step: float) -> PathStepper:
        limit = np.full(path.shape[1], step)
        limit[self.robot.torso] = step * self.settings.torso_scale
        return PathStepper(path, limit)

    def _fail(self, reason: str) -> tuple[np.ndarray, Status]:
        self.failure = reason
        return self._target.copy(), Status.FAILED

    def _arrived(self, obs) -> Optional[bool]:
        """True once the arm is at the end of the current path, None while it
        is still settling, False if it does not get there."""
        joints = list(self.robot.plan_joints)
        error = np.abs(np.asarray(obs["qpos"], dtype=float)[joints] - self._target[joints]).max()
        if error < self.settings.arrive_tolerance:
            self._waited = 0
            return True
        self._waited += 1
        return None if self._waited <= self.settings.arrive_ticks else False

    def _object_moved(self, obs) -> bool:
        gap = np.asarray(obs["object_pos"], dtype=float) - self.plan.object_position
        return bool(np.linalg.norm(gap) > self.settings.moved_tolerance)

    def step(self, obs) -> tuple[np.ndarray, Status]:
        """The next (15,) action. FAILED if the object moves before the close,
        if the hold is not verified after it, or if the object is lost in the
        lift or the hold. DONE after the object has been held `hold_ticks`
        at or above `lift_height`."""
        s, robot = self.settings, self.robot
        fingers = list(robot.fingers)

        if self.phase in ("transit", "approach"):
            if self._object_moved(obs) and self.plan.candidate.kind != "rake":
                return self._fail("object moved before the close")
            if not self._stepper.finished:
                self._target = self._stepper.step()
                return self._target.copy(), Status.RUNNING
            arrived = self._arrived(obs)
            if arrived is None:
                return self._target.copy(), Status.RUNNING
            if not arrived:
                return self._fail(f"arm did not reach the end of the {self.phase}")
            if self.phase == "transit":
                self.phase = "approach"
                step = s.tight_step if self.plan.candidate.kind == "tight" else s.approach_step
                self._stepper = self._stepper_for(self.plan.approach, step)
            elif self.plan.candidate.kind == "rake":
                self.phase = "gripped"                   # nothing to close on: the hand is in place to pull
                return self._target.copy(), Status.DONE
            else:
                self.phase = "close"
            return self._target.copy(), Status.RUNNING

        if self.phase == "close":
            if self._object_moved(obs) and not any(finger_contacts(self.env, robot, s.contact_force)):
                return self._fail("object moved before the close")
            touching = finger_contacts(self.env, robot, s.contact_force)
            if all(touching):
                # Squeeze: put the targets a fixed distance inside the object.
                measured = np.asarray(obs["qpos"], dtype=float)[fingers]
                self._target[fingers] = np.maximum(robot.lower[fingers], measured - s.squeeze)
                self.phase, self._counter = "grip", 0
            elif np.all(self._target[fingers] <= robot.lower[fingers] + 1e-9):
                self._counter += 1
                if self._counter > s.grip_ticks:
                    return self._fail("fingers closed on nothing")
            else:
                self._target[fingers] = np.maximum(robot.lower[fingers], self._target[fingers] - s.close_step)
            return self._target.copy(), Status.RUNNING

        held = is_held(self.env, robot, s.contact_force)
        self._lost = 0 if held else self._lost + 1

        if self.phase == "grip":
            if self._lost >= s.lost_ticks:
                return self._fail("hold not verified after the close")
            self._counter = self._counter + 1 if held else 0
            if self._counter >= s.grip_ticks:
                self.phase, self._counter = "lift", 0
                self._stepper = self._stepper_for(self._with_fingers(self.plan.lift), s.lift_step)
            return self._target.copy(), Status.RUNNING

        if self._lost >= s.lost_ticks:
            return self._fail(f"object lost during the {self.phase}")
        risen = float(obs["object_pos"][2]) - self._rest_height

        if self.phase == "lift":
            self._counter += 1
            if risen >= s.lift_goal:                     # high enough: stop here
                self.lifted_tick_count = self._counter
                self.phase, self._counter = "hold", 0
                return self._target.copy(), Status.RUNNING
            if not self._stepper.finished:
                self._target = self._stepper.step()
                return self._target.copy(), Status.RUNNING
            arrived = self._arrived(obs)
            if arrived is None:
                return self._target.copy(), Status.RUNNING
            if not arrived:
                return self._fail("arm did not reach the end of the lift")
            if risen < self._lift_height:
                return self._fail(f"object rose {risen:.3f} m, less than the lift height")
            self.lifted_tick_count = self._counter
            self.phase, self._counter = "hold", 0
            return self._target.copy(), Status.RUNNING

        # hold
        if risen < self._lift_height:
            return self._fail("object sank below the lift height during the hold")
        self._counter += 1
        return self._target.copy(), (Status.DONE if self._counter >= self._hold_ticks else Status.RUNNING)

    def _with_fingers(self, path: np.ndarray) -> np.ndarray:
        """The path with the finger targets as they are now (closed on the object)."""
        path = np.array(path, dtype=float)
        path[:, list(self.robot.fingers)] = self._target[list(self.robot.fingers)]
        return path
