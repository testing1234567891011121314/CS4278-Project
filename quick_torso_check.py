"""Bounded check: is the grasp IK reachable only if torso_lift is free?
Replays the step-59 parked pose, IK to the measured ee_target, torso pinned vs free."""
import numpy as np
import pybullet as p
import pybullet_data

cid = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
p.setGravity(0, 0, 0, physicsClientId=cid)
p.loadURDF("plane.urdf", physicsClientId=cid)
robot = p.loadURDF("assets/fetch/fetch.urdf", [-2, 0, 0], useFixedBase=True, physicsClientId=cid)

PREFIXES = (b"shoulder", b"upperarm", b"elbow", b"forearm", b"wrist")
movable, arm, torso = [], [], []
for j in range(p.getNumJoints(robot, physicsClientId=cid)):
    info = p.getJointInfo(robot, j, physicsClientId=cid)
    if info[2] not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        continue
    movable.append(j)
    if info[1].startswith(PREFIXES):
        arm.append(j)
    if info[1] == b"torso_lift_joint":
        torso.append(j)
ee = [j for j in range(p.getNumJoints(robot, physicsClientId=cid))
      if p.getJointInfo(robot, j, physicsClientId=cid)[12] == b"gripper_link"][0]

# The parked pose at failure (from step 59 log), root joints at their qpos.
current = np.zeros(len(movable))
# qpos order: root x, y, yaw, torso, head x2, arm x7, fingers x2
qpos59 = [0.928, -0.035, -0.001, 0.0, 0.0, 0.0,
          0.0035, -0.9598, 0.0472, -2.0554, 0.2600, 1.0216, -0.0291,
          0.0, 0.0]
for j, v in zip(movable, qpos59):
    p.resetJointState(robot, j, float(v), physicsClientId=cid)

# Measured best-candidate world ee_target (robot anchored at [-2,0,0], root
# joints carry it to world x = -2+0.928 = -1.072).
ee_target = [-0.105, -0.028, 0.656]
# Wrist pointing straight down (approach = -z), closing axis horizontal y.
x_axis = np.array([0.0, 0.0, -1.0])
y_axis = np.array([0.0, 1.0, 0.0])
z_axis = np.cross(x_axis, y_axis)
R = np.stack([x_axis, y_axis, z_axis], axis=1)
# mat -> quat (xyzw)
tr = np.trace(R)
s = np.sqrt(tr + 1.0) * 2
quat = np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
quat /= np.linalg.norm(quat)

arm_pos = [movable.index(j) for j in arm]
torso_pos = [movable.index(j) for j in torso]


def try_ik(free_torso):
    lower, upper = current.copy(), current.copy()
    free = arm + (torso if free_torso else [])
    for j in free:
        k = movable.index(j)
        info = p.getJointInfo(robot, j, physicsClientId=cid)
        lo, hi = float(info[8]), float(info[9])
        if hi >= lo:
            lower[k], upper[k] = lo, hi
    rest = current.copy()
    rest[arm_pos] = [0.0, 0.3, 0.0, -0.9, 0.0, 0.6, 0.0]
    sol = np.asarray(p.calculateInverseKinematics(
        robot, ee, targetPosition=ee_target, targetOrientation=quat.tolist(),
        lowerLimits=lower.tolist(), upperLimits=upper.tolist(),
        jointRanges=(upper - lower).tolist(), restPoses=rest.tolist(),
        maxNumIterations=200, residualThreshold=1e-4, physicsClientId=cid))
    free_pos = arm_pos + (torso_pos if free_torso else [])
    out = (sol[free_pos] < lower[free_pos] - 1e-6) | (sol[free_pos] > upper[free_pos] + 1e-6)
    # FK truth
    for j, v in zip(movable, current):
        p.resetJointState(robot, j, float(v), physicsClientId=cid)
    for j, v in zip(free, sol[free_pos]):
        p.resetJointState(robot, j, float(v), physicsClientId=cid)
    st = p.getLinkState(robot, ee, computeForwardKinematics=True, physicsClientId=cid)
    err = np.linalg.norm(np.asarray(st[4]) - np.asarray(ee_target))
    print(f"torso_free={free_torso}: out_of_limits={int(out.sum())} "
          f"joints_out={[arm[i] if i < len(arm_pos) else 'torso' for i in np.flatnonzero(out)][:4]} "
          f"torso_sol={sol[torso_pos[0]]:.3f} FK_pos_err={err:.4f} m")


try_ik(False)
try_ik(True)
