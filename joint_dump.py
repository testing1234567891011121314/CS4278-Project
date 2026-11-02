import pybullet as p, pybullet_data
cid = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
r = p.loadURDF("assets/fetch/fetch.urdf", [0,0,0], useFixedBase=True, physicsClientId=cid)
PREFIXES = (b"shoulder", b"upperarm", b"elbow", b"forearm", b"wrist")
movable, arm = [], []
for j in range(p.getNumJoints(r, physicsClientId=cid)):
    info = p.getJointInfo(r, j, physicsClientId=cid)
    if info[2] not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        continue
    movable.append(j)
    is_arm = info[1].startswith(PREFIXES)
    if is_arm: arm.append(j)
    print(j, info[1].decode(), "type", info[2], "lo", round(float(info[8]),4), "hi", round(float(info[9]),4), "ARM" if is_arm else "")
print("movable:", movable)
print("arm:", arm)
