import numpy as np, pybullet as p, pybullet_data
cid = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
r = p.loadURDF("assets/fetch/fetch.urdf", [0,0,0], useFixedBase=True, physicsClientId=cid)
n = p.getNumJoints(r, physicsClientId=cid)
for j in range(n):
    info = p.getJointInfo(r, j, physicsClientId=cid)
    if b"gripper" in info[1] or b"gripper" in info[12]:
        print(j, info[1].decode(), "type", info[2], "axis", np.round(info[2],2) if False else info[5], "child", info[12].decode())
# FK: set a known arm pose and see where gripper_link + finger tips sit relative to gripper frame
grip = [j for j in range(n) if p.getJointInfo(r,j,physicsClientId=cid)[12]==b"gripper_link"][0]
st = p.getLinkState(r, grip, computeForwardKinematics=True, physicsClientId=cid)
gp, gq = st[4], st[5]
print("gripper_link world pos:", np.round(gp,3))
for j in range(n):
    info = p.getJointInfo(r, j, physicsClientId=cid)
    if info[12] in (b"r_gripper_finger_link", b"l_gripper_finger_link"):
        s = p.getLinkState(r, j, computeForwardKinematics=True, physicsClientId=cid)
        fp = np.asarray(s[4]) - np.asarray(gp)
        # rotate into gripper frame
        from pybullet import getMatrixFromQuaternion, invertTransform, multiplyTransforms
        R = np.array(getMatrixFromQuaternion(gq)).reshape(3,3)
        local = R.T @ fp
        print(info[12].decode(), "offset in gripper frame:", np.round(local,3))
# open the fingers fully and re-measure
for j in range(n):
    info = p.getJointInfo(r, j, physicsClientId=cid)
    if info[1] in (b"r_gripper_finger_joint", b"l_gripper_finger_joint"):
        p.resetJointState(r, j, 0.05, physicsClientId=cid)
for j in range(n):
    info = p.getJointInfo(r, j, physicsClientId=cid)
    if info[12] in (b"r_gripper_finger_link", b"l_gripper_finger_link"):
        s = p.getLinkState(r, j, computeForwardKinematics=True, physicsClientId=cid)
        st = p.getLinkState(r, grip, computeForwardKinematics=True, physicsClientId=cid)
        fp = np.asarray(s[4]) - np.asarray(st[4])
        from pybullet import getMatrixFromQuaternion
        R = np.array(getMatrixFromQuaternion(st[5])).reshape(3,3)
        print(info[12].decode(), "offset OPEN:", np.round(R.T @ fp,3))
