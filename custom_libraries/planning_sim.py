"""The solver's own simulator for planning (R30, R31).

A second physics client holding a copy of the scene. Planning code sets poses
here and asks for distances; the env's simulator is only ever read. See
docs/adr/0001-plan-in-a-second-simulator.md.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import pybullet as p

from custom_libraries.robot import RobotModel


SHAPES_PER_BODY = 16     # the simulator builds a body from at most this many collision shapes


def rotation(quaternion) -> np.ndarray:
    """3x3 rotation matrix of an xyzw quaternion."""
    return np.array(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)


@dataclass(frozen=True)
class Gripper:
    """Dimensions of the hand, in the frame of the link the fingers ride on:
    x points from the palm out along the fingers, y from one pad to the other."""

    max_opening: float      # clear gap between the pads, fully open
    min_opening: float      # clear gap, fully closed
    finger_travel: float    # joint travel of one finger
    pad_back: float         # x where the pads start
    pad_tip: float          # x of the fingertips
    pad_width: float        # pad extent across x and y (along z)
    pad_thickness: float    # finger extent behind the pad surface (along y)
    palm_front: float       # x of the palm's front face
    palm_half: tuple[float, float]    # the palm's half extents along y and z

    def finger_joint(self, opening: float) -> float:
        """Joint value of each finger for a clear gap of `opening`."""
        return float(np.clip((opening - self.min_opening) / 2.0, 0.0, self.finger_travel))


class PlanningSim:
    def __init__(self, env):
        """Open a second physics client and load a copy of the env's scene,
        found by reading the env's client. Nothing is written there."""
        self.robot = RobotModel.from_env(env)
        live = self.robot.client_id
        # Folders a relative mesh path may be relative to: where we are now,
        # where the env's code is, and the project this file belongs to.
        module = sys.modules.get(type(env).__module__)
        self._roots = [Path.cwd(), Path(__file__).resolve().parents[1]]
        if getattr(module, "__file__", None):
            self._roots.insert(1, Path(module.__file__).resolve().parent)
        self.client = p.connect(p.DIRECT)
        if self.client < 0:
            raise RuntimeError("could not open the planning simulator")
        try:
            self.robot_id = self._copy_articulated(live, env.robot_id, fixed=True)
            self.object_id = self._copy_articulated(live, env.object_id, fixed=False)
            self.static_ids: list[int] = []             # every fixed body of the copy
            self.copies_of: dict[int, list[int]] = {}   # live body -> the bodies that copy it
            self.floor_id: Optional[int] = None
            for index in range(p.getNumBodies(physicsClientId=live)):
                body = p.getBodyUniqueId(index, physicsClientId=live)
                if body in (env.robot_id, env.object_id):
                    continue
                copies = self._copy_static(live, body)
                if not copies:
                    continue
                self.copies_of[body] = copies
                self.static_ids.extend(copies)
                upper = p.getAABB(copies[0], physicsClientId=self.client)[1]
                if len(copies) == 1 and upper[2] < 0.05 and self.floor_id is None:
                    self.floor_id = copies[0]
            self._joint_count = p.getNumJoints(self.robot_id, physicsClientId=self.client)
            self._q = np.zeros(len(self.robot.joint_indices))
            self.set_robot(self._q)
            self._check_copy(live, env)
            self.gripper = self._measure_gripper()
            self.base_radius = self._measure_base()
            self._hand = self._build_hand()
        except Exception:
            self.close()
            raise

    # ------------------------------------------------------------ construction

    def _copy_articulated(self, live: int, body: int, fixed: bool) -> int:
        """Load the description file the live body came from. It is found next
        to (or above) one of the body's mesh files, and recognised by having
        the same joints."""
        names = [p.getJointInfo(body, j, physicsClientId=live)[1] for j in
                 range(p.getNumJoints(body, physicsClientId=live))]
        meshes = [s[4].decode() for s in p.getVisualShapeData(body, physicsClientId=live) if s[4]]
        meshes += [s[4].decode() for s in p.getCollisionShapeData(body, -1, physicsClientId=live) if s[4]]
        position, orientation = p.getBasePositionAndOrientation(body, physicsClientId=live)
        tried = set()
        for mesh in self._existing(meshes):
            folder = mesh.parent
            for _ in range(4):
                for path in sorted(folder.glob("*.urdf")):
                    if path in tried:
                        continue
                    tried.add(path)
                    copy = p.loadURDF(str(path), position, orientation, useFixedBase=fixed,
                                      physicsClientId=self.client)
                    same = [p.getJointInfo(copy, j, physicsClientId=self.client)[1] for j in
                            range(p.getNumJoints(copy, physicsClientId=self.client))] == names
                    if same and self._same_shapes(live, body, copy):
                        return copy
                    p.removeBody(copy, physicsClientId=self.client)
                folder = folder.parent
        raise RuntimeError(f"no description file found for body {body} near {meshes[:1]}")

    def _existing(self, meshes: list[str]) -> list[Path]:
        """The mesh files as absolute paths. The simulator reports them as
        they were given when the scene was loaded, often relative to the
        folder the process was in then, which may no longer be the current one."""
        found = []
        for mesh in meshes:
            for root in self._roots:
                path = (root / mesh).resolve()
                if path.is_file():
                    found.append(path)
                    break
        return found

    def _same_shapes(self, live: int, body: int, copy: int) -> bool:
        a = p.getCollisionShapeData(body, -1, physicsClientId=live)
        b = p.getCollisionShapeData(copy, -1, physicsClientId=self.client)
        return [(s[2], s[3], Path(s[4].decode()).name) for s in a] == \
               [(s[2], s[3], Path(s[4].decode()).name) for s in b]

    def _copy_static(self, live: int, body: int) -> list[int]:
        """Rebuild a fixed body from its collision shapes, as one or more
        bodies (the simulator limits how many shapes one body may be built
        from). Empty if the body has no collision shape."""
        try:
            shapes = p.getCollisionShapeData(body, -1, physicsClientId=live)
        except p.error:                 # a body with no collision shape, such as the goal marker
            return []
        if not shapes:
            return []
        if p.getNumJoints(body, physicsClientId=live) or \
                p.getDynamicsInfo(body, -1, physicsClientId=live)[0] != 0.0:
            raise RuntimeError(f"body {body} is not a single fixed link; it cannot be copied")
        position, orientation = p.getBasePositionAndOrientation(body, physicsClientId=live)
        return [self._build_static(shapes[i:i + SHAPES_PER_BODY], position, orientation)
                for i in range(0, len(shapes), SHAPES_PER_BODY)]

    def _build_static(self, shapes, position, orientation) -> int:
        kinds, half, radii, heights, files, scales, positions, orientations = [], [], [], [], [], [], [], []
        for s in shapes:
            kind, dims = s[2], s[3]
            kinds.append(kind)
            half.append([d / 2.0 for d in dims] if kind == p.GEOM_BOX else [1.0, 1.0, 1.0])
            radii.append(dims[0] if kind == p.GEOM_SPHERE else dims[1] if kind in (p.GEOM_CYLINDER, p.GEOM_CAPSULE)
                         else 0.5)
            heights.append(dims[0] if kind in (p.GEOM_CYLINDER, p.GEOM_CAPSULE) else 1.0)
            files.append(s[4].decode())
            scales.append(list(dims) if kind == p.GEOM_MESH else [1.0, 1.0, 1.0])
            positions.append(s[5])
            orientations.append(s[6])
        shape = p.createCollisionShapeArray(
            shapeTypes=kinds, halfExtents=half, radii=radii, lengths=heights, fileNames=files, meshScales=scales,
            collisionFramePositions=positions, collisionFrameOrientations=orientations,
            physicsClientId=self.client)
        return p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shape, basePosition=position,
                                 baseOrientation=orientation, physicsClientId=self.client)

    def _check_copy(self, live: int, env) -> None:
        """The copy must be the same robot in the same place as the live one."""
        count = p.getNumJoints(env.robot_id, physicsClientId=live)
        if count != self._joint_count:
            raise RuntimeError("the planning robot has a different number of joints")
        q = np.array([s[0] for s in p.getJointStates(env.robot_id, list(self.robot.joint_indices),
                                                     physicsClientId=live)])
        self.set_robot(q)
        link = self.robot.gripper_link
        a = p.getLinkState(env.robot_id, link, computeForwardKinematics=True, physicsClientId=live)
        b = p.getLinkState(self.robot_id, link, computeForwardKinematics=True, physicsClientId=self.client)
        if np.linalg.norm(np.subtract(a[4], b[4])) > 1e-5:
            raise RuntimeError("the planning robot's gripper is not where the live one is")

    def _measure_gripper(self) -> Gripper:
        robot, links = self.robot, self.robot.finger_links
        travel = float(min(robot.upper[k] - robot.lower[k] for k in robot.fingers))

        def gap(value: float) -> float:
            q = self._q.copy()
            q[list(robot.fingers)] = value
            self.set_robot(q)
            points = p.getClosestPoints(self.robot_id, self.robot_id, 1.0, linkIndexA=links[0],
                                        linkIndexB=links[1], physicsClientId=self.client)
            return float(min(c[8] for c in points))

        opened, closed = gap(travel), gap(0.0)
        # Shapes in the frame of the link the fingers ride on, at zero finger travel.
        position, orientation = self.link_pose(robot.gripper_link)
        frame = rotation(orientation)

        def corners(link: int) -> np.ndarray:
            """Corners of the link's collision shapes, in the gripper frame."""
            state = p.getLinkState(self.robot_id, link, computeForwardKinematics=True, physicsClientId=self.client)
            out = []
            for s in p.getCollisionShapeData(self.robot_id, link, physicsClientId=self.client):
                if s[2] == p.GEOM_BOX:      # the shape's frame is given relative to the link's centre of mass
                    centre, turn = p.multiplyTransforms(state[0], state[1], s[5], s[6])
                    half = np.array(s[3]) / 2.0
                    local = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
                else:                       # mesh vertices come already in the centre-of-mass frame
                    centre, turn = state[0], state[1]
                    local = np.array(p.getMeshData(self.robot_id, link, physicsClientId=self.client)[1])
                out.append(local @ rotation(turn).T + np.array(centre))
            return (np.vstack(out) - np.array(position)) @ frame

        pad = corners(links[0])
        palm = corners(robot.gripper_link)
        return Gripper(
            max_opening=opened, min_opening=closed, finger_travel=travel,
            pad_back=float(pad[:, 0].min()), pad_tip=float(pad[:, 0].max()),
            pad_width=float(np.ptp(pad[:, 2])), pad_thickness=float(np.ptp(pad[:, 1])),
            palm_front=float(palm[:, 0].max()),
            palm_half=(float(np.abs(palm[:, 1]).max()), float(np.abs(palm[:, 2]).max())),
        )

    def _measure_base(self) -> float:
        """Radius of the circle round the base centre that holds the base link."""
        robot = self.robot
        self.set_robot(np.zeros(len(robot.joint_indices)))
        lower, upper = p.getAABB(self.robot_id, robot.joint_indices[robot.base[2]], physicsClientId=self.client)
        centre = robot.base_origin[:2]
        return float(max(np.abs(np.subtract(lower[:2], centre)).max(), np.abs(np.subtract(upper[:2], centre)).max()))

    def _build_hand(self) -> dict:
        """Loose copies of the palm and the two fingers, so the hand alone can
        be placed anywhere and checked without solving for the arm."""
        robot = self.robot
        self.set_robot(np.zeros(len(robot.joint_indices)))
        origin, turn = self.link_pose(robot.gripper_link)
        frame = rotation(turn)

        def body(link: int) -> int:
            """A fixed body with the link's collision shapes, placed about the link's own frame."""
            state = p.getLinkState(self.robot_id, link, computeForwardKinematics=True, physicsClientId=self.client)
            shapes = []
            for s in p.getCollisionShapeData(self.robot_id, link, physicsClientId=self.client):
                position, orientation = p.multiplyTransforms(state[2], state[3], s[5], s[6])
                if s[2] == p.GEOM_BOX:
                    shapes.append(p.createCollisionShape(
                        p.GEOM_BOX, halfExtents=[d / 2.0 for d in s[3]], collisionFramePosition=position,
                        collisionFrameOrientation=orientation, physicsClientId=self.client))
                else:
                    shapes.append(p.createCollisionShape(
                        p.GEOM_MESH, fileName=s[4].decode(), meshScale=list(s[3]),
                        collisionFramePosition=position, collisionFrameOrientation=orientation,
                        physicsClientId=self.client))
            if len(shapes) != 1:
                raise RuntimeError(f"link {link} has {len(shapes)} collision shapes; one was expected")
            return p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=shapes[0], basePosition=[0, 0, -50.0],
                                     physicsClientId=self.client)

        def in_hand(link: int) -> np.ndarray:
            """Position of the link's frame in the gripper frame, for the finger travel last set."""
            return frame.T @ (self.link_pose(link)[0] - origin)

        closed = [in_hand(link) for link in robot.finger_links]
        q = np.zeros(len(robot.joint_indices))
        q[list(robot.fingers)] = robot.upper[list(robot.fingers)]
        self.set_robot(q)
        opened = [in_hand(link) for link in robot.finger_links]
        travel = float(robot.upper[robot.fingers[0]] - robot.lower[robot.fingers[0]])
        return dict(palm=body(robot.gripper_link), fingers=[body(link) for link in robot.finger_links],
                    closed=closed, slide=[(o - c) / travel for o, c in zip(opened, closed)])

    def hand_clearance(self, position, turn: np.ndarray, opening: float, others, look: float = 0.03) -> tuple:
        """Smallest gaps in metres between the hand alone, placed with its
        frame at the world `position` and 3x3 rotation `turn` and opened to a
        clear gap of `opening`, and (the object, the bodies in `others`).
        Gaps beyond `look` are reported as `look`."""
        from custom_libraries.collision import _quaternion
        orientation = _quaternion(turn).tolist()
        value = self.gripper.finger_joint(opening)
        parts = [(self._hand["palm"], np.zeros(3))]
        parts += [(body, closed + slide * value) for body, closed, slide in
                  zip(self._hand["fingers"], self._hand["closed"], self._hand["slide"])]
        to_object, to_others = look, look
        for body, offset in parts:
            p.resetBasePositionAndOrientation(body, (np.asarray(position) + turn @ offset).tolist(), orientation,
                                              physicsClientId=self.client)
            for c in p.getClosestPoints(body, self.object_id, look, physicsClientId=self.client):
                to_object = min(to_object, c[8])
            for other in others:
                for c in p.getClosestPoints(body, other, look, physicsClientId=self.client):
                    to_others = min(to_others, c[8])
        for body, _ in parts:                       # out of the way of every other query
            p.resetBasePositionAndOrientation(body, [0, 0, -50.0], [0, 0, 0, 1], physicsClientId=self.client)
        return to_object, to_others

    def wedge(self, position, turn: np.ndarray, opening: float, look: float = 0.03) -> float:
        """How far from parallel the two surfaces are that the pads would
        close on, in radians, with the hand placed as for `hand_clearance`:
        the angle between the object's surface normal nearest one finger and
        the reversed normal nearest the other. The pads squeeze a wedge out
        of the hand once half this angle passes the friction angle."""
        from custom_libraries.collision import _quaternion
        orientation = _quaternion(turn).tolist()
        value = self.gripper.finger_joint(opening)
        normals = []
        for body, closed, slide in zip(self._hand["fingers"], self._hand["closed"], self._hand["slide"]):
            p.resetBasePositionAndOrientation(body, (np.asarray(position) + turn @ (closed + slide * value)).tolist(),
                                              orientation, physicsClientId=self.client)
            points = p.getClosestPoints(body, self.object_id, look, physicsClientId=self.client)
            p.resetBasePositionAndOrientation(body, [0, 0, -50.0], [0, 0, 0, 1], physicsClientId=self.client)
            if not points:
                return 0.0
            normals.append(np.array(min(points, key=lambda c: c[8])[7]))
        cos = float(np.clip(normals[0] @ -normals[1], -1.0, 1.0))
        return float(np.arccos(cos))

    # ------------------------------------------------------------------- state

    def set_robot(self, qpos) -> None:
        """Place the copy's robot at the joint values `qpos`."""
        self._q = np.array(qpos, dtype=float)
        for index, value in zip(self.robot.joint_indices, self._q):
            p.resetJointState(self.robot_id, index, float(value), physicsClientId=self.client)

    def set_object(self, position, orientation) -> None:
        """Place the copy's object at a world pose, at rest."""
        p.resetBasePositionAndOrientation(self.object_id, [float(v) for v in position],
                                          [float(v) for v in orientation], physicsClientId=self.client)
        p.resetBaseVelocity(self.object_id, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)

    def sync(self, obs) -> None:
        """Set the copy's robot joints and object pose to the observation."""
        self.set_robot(obs["qpos"])
        self.set_object(obs["object_pos"], obs["object_xyzw"])

    def link_pose(self, link: int) -> tuple[np.ndarray, np.ndarray]:
        """World position and xyzw quaternion of a robot link's frame, for the
        joint values last set."""
        state = p.getLinkState(self.robot_id, link, computeForwardKinematics=True, physicsClientId=self.client)
        return np.array(state[4]), np.array(state[5])

    def gripper_pose(self, qpos) -> tuple[np.ndarray, np.ndarray]:
        """World position and 3x3 rotation of the gripper frame at `qpos`."""
        self.set_robot(qpos)
        position, orientation = self.link_pose(self.robot.gripper_link)
        return position, rotation(orientation)

    def close(self) -> None:
        """Disconnect the second client."""
        if self.client >= 0 and p.isConnected(self.client):
            p.disconnect(self.client)
        self.client = -1
