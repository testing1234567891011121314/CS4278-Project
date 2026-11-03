import numpy as np, pybullet as p, pybullet_data
cid = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
r = p.loadURDF("assets/fetch/fetch.urdf", [0,0,0], useFixedBase=True, physicsClientId=cid)
n = p.getNumJoints(r, physicsClientId=cid)
grip = [j for j in range(n) if p.getJointInfo(r,j,physicsClientId=cid)[12]==b"gripper_link"][0]
st = p.getLinkState(r, grip, computeForwardKinematics=True, physicsClientId=cid)
gp = np.asarray(st[4]); R = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3,3)
print("spawn arm-zero pose. gripper_link world pos:", np.round(gp,3))
print("  x-axis:", np.round(R[:,0],2), "y:", np.round(R[:,1],2), "z:", np.round(R[:,2],2))
for j in range(n):
    info = p.getJointInfo(r,j,physicsClientId=cid)
    if info[12] in (b"r_gripper_finger_link", b"l_gripper_finger_link"):
        s = p.getLinkState(r,j,computeForwardKinematics=True,physicsClientId=cid)
        off = np.asarray(s[4]) - gp
        print(info[12].decode(), "world offset from gripper origin:", np.round(off,3),
              " in gripper frame:", np.round(R.T@off,3))
# Now command MY convention for a top-down grasp and FK it with huge limits.
axis = np.array([1.0,0.0,0.0]); descent = np.array([0.0,0.0,-1.0])
x = descent; y = axis; z = np.cross(x,y)
Rm = np.stack([x,y,z],axis=1)
tr = np.trace(Rm); s_ = np.sqrt(tr+1)*2
q = np.array([(Rm[2,1]-Rm[1,2])/s_, (Rm[0,2]-Rm[2,0])/s_, (Rm[1,0]-Rm[0,1])/s_, 0.25*s_]); q/=np.linalg.norm(q)
print("my quat (xyzw):", np.round(q,3))
arm = [j for j in range(n) if p.getJointInfo(r,j,physicsClientId=cid)[1].startswith(
    (b"shoulder",b"upperarm",b"elbow",b"forearm",b"wrist"))]
target = gp + np.array([0.0,0.0,-0.15])
movable = [j for j in range(n) if p.getJointInfo(r,j,physicsClientId=cid)[2] in (0,1)]
cur = np.array([p.getJointState(r,j,physicsClientId=cid)[0] for j in movable])
ap = [movable.index(j) for j in arm]
lo, hi = cur.copy(), cur.copy()
for k,j in enumerate(arm): lo[ap[k]], hi[ap[k]] = -10, 10
sol = np.array(p.calculateInverseKinematics(r, grip, targetPosition=target.tolist(),
    targetOrientation=q.tolist(), lowerLimits=lo.tolist(), upperLimits=hi.tolist(),
    jointRanges=(hi-lo).tolist(), restPoses=cur.tolist(), maxNumIterations=500, residualThreshold=1e-6, physicsClientId=cid))
print("huge-limit IK arm sol:", np.round(sol[ap],2))
for j,v in zip(arm, sol[ap]): p.resetJointState(r,j,float(v),physicsClientId=cid)
st2 = p.getLinkState(r, grip, computeForwardKinematics=True, physicsClientId=cid)
R2 = np.array(p.getMatrixFromQuaternion(st2[5])).reshape(3,3)
print("FK achieved pos err:", round(float(np.linalg.norm(np.asarray(st2[4])-target)),4))
print("FK x-axis:", np.round(R2[:,0],2), "y:", np.round(R2[:,1],2), "(want x=down, y=+x_world)")
