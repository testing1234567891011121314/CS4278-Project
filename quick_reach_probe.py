"""End-to-end planner probe with the custom solve_ik: settle, park the base
0.70 m from the can with the env's motors, run real select_optimal_grasp,
print per-candidate failure points, visualize the winner."""
import time
import numpy as np
import pybullet as p
from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.antipodal import AntipodalGraspPlanner

env = PickNavReachEnv(seed=42, gui=True)
obs = env.reset()
for _ in range(200):
    obs, _ = env.step(np.zeros(env.action_size))
client, robot, obj_id = env.pb_physics_client, env.robot_id, env.object_id
obj = np.asarray(obs["object_pos"])
print(f"OBJECT: {np.round(obj, 3)}", flush=True)

n = p.getNumJoints(robot, physicsClientId=client)
ee = [j for j in range(n)
      if p.getJointInfo(robot, j, physicsClientId=client)[12] == b"gripper_link"][0]

D, TOR = 0.70, 0.0
target = np.zeros(env.action_size)
target[[0, 1, 2, 5]] = [obj[0] - D + 2.0, obj[1], 0.0, TOR]
for _ in range(300):
    obs, _ = env.step(target)
qpos = np.asarray(obs["qpos"])
print(f"BASE settled: {obj[0] - (qpos[0] - 2.0):.3f} m from object "
      f"(want {D}), torso={qpos[5]:.2f}", flush=True)

aabb = p.getAABB(obj_id, -1, physicsClientId=client)
print(f"OBJECT AABB min={np.round(aabb[0], 3)} max={np.round(aabb[1], 3)}", flush=True)
print(f"PC extent min={np.round(obs['object_pc'].min(axis=0), 3)} "
      f"max={np.round(obs['object_pc'].max(axis=0), 3)}", flush=True)

planner = AntipodalGraspPlanner(physics_client_id=client)
cands = planner.sample_antipodal_grasps(obj_id, points=obs["object_pc"],
                                        normals=obs["object_normals"])
print(f"candidates: {len(cands)}", flush=True)

import time as _t
t0 = _t.time()
g = planner.select_optimal_grasp(robot, ee, cands, strategy="score_all",
                                 support_z=float(np.min(obs["object_pc"][:, 2])))
st = planner.ik_stats["score_all"]
print(f"grasp: {'FOUND' if g else 'NONE'} calls={st['calls']} "
      f"ik_fail={st['ik_fail']} fk_fail={st['fk_fail']} in {_t.time()-t0:.1f}s", flush=True)
for r in st["reasons"]:
    print(f"  reason: {r}", flush=True)
for f in st["failures"][:5]:
    print(f"  failure@{f['center']}: pos_err={f['pos_err']}m "
          f"axis_err={f['axis_err_deg']}deg arm={f['arm']}", flush=True)
def _qmat(q):
    from custom_libraries.antipodal import _quat_to_mat
    return _quat_to_mat(np.asarray(q, float))

if g:
    p1w, p2w = np.asarray(g["p1"], float), np.asarray(g["p2"], float)
    ax = p2w - p1w
    print(f"  p1={np.round(p1w,3)} p2={np.round(p2w,3)} axis={np.round(ax/np.linalg.norm(ax),2)}",
          flush=True)
    print(f"  score={g['score']:.3f} arm={np.round(g['joint_angles'], 2)}", flush=True)
    print(f"  ee_target={np.round(g['ee_target'], 3)}", flush=True)
    goal = target.copy()
    goal[-2:] = env.joint_upper[-2:]  # fingers OPEN during descent, like solver's open_pose
    arm_names = ("shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                 "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint",
                 "wrist_roll_joint")
    name_to_joint = {p.getJointInfo(robot, j, physicsClientId=client)[1].decode(): j
                     for j in range(n)}
    for k, name in enumerate(arm_names):
        goal[6 + k] = float(g["joint_angles"][k])
    for _ in range(200):
        env.step(goal)
    time.sleep(2.0)
    ls = p.getLinkState(robot, ee, computeForwardKinematics=True, physicsClientId=client)
    print(f"  VISUALIZED ee={np.round(ls[4], 3)} target={np.round(g['ee_target'], 3)}", flush=True)
    table = [b for b in range(p.getNumBodies(physicsClientId=client))
             if b not in (robot, obj_id)]
    for b in table:
        name = p.getBodyInfo(b, physicsClientId=client)[0].decode()
        pts = p.getContactPoints(bodyA=robot, bodyB=b, physicsClientId=client)
        if pts:
            zs = np.array([pt[5] for pt in pts])
            print(f"  CONTACT robot-vs-{name}: {len(pts)} pts z=[{zs.min():.3f},{zs.max():.3f}]",
                  flush=True)
    pts = p.getContactPoints(bodyA=robot, bodyB=obj_id, physicsClientId=client)
    if pts:
        zs = np.array([pt[5] for pt in pts])
        print(f"  CONTACT robot-vs-object: {len(pts)} pts z=[{zs.min():.3f},{zs.max():.3f}]",
              flush=True)
    # Close the fingers like solver's close_pose and see if the can comes along.
    obj_before = np.asarray(p.getBasePositionAndOrientation(obj_id,
                              physicsClientId=client)[0])
    # Ramp the close: hard force-2e6 steps slam the light can out from
    # between the pads; gentle approach keeps it centred.
    for f in np.linspace(0.05, float(env.joint_lower[-2]), 40):
        close_goal = goal.copy()
        close_goal[-2:] = f
        for _ in range(8):
            env.step(close_goal)
    obj_after = np.asarray(p.getBasePositionAndOrientation(obj_id,
                             physicsClientId=client)[0])
    pts = p.getContactPoints(bodyA=robot, bodyB=obj_id, physicsClientId=client)
    print(f"  CLOSED: fingers={np.round(p.getJointStates(robot, [21, 22], physicsClientId=client)[0][0], 4)}"
          f" obj moved {np.round(obj_after - obj_before, 3)} "
          f"object contacts={len(pts)}", flush=True)
    lift_goal = goal.copy()
    lift_goal[-2:] = env.joint_lower[-2:]
    for tz in np.linspace(float(goal[5]), float(env.joint_upper[5]), 30):
        lift_goal[5] = tz  # torso lift carries the arm up smoothly
        for _ in range(10):
            env.step(lift_goal)
    obj_lift = np.asarray(p.getBasePositionAndOrientation(obj_id,
                            physicsClientId=client)[0])
    print(f"  LIFT: obj z {obj_before[2]:.3f} -> {obj_lift[2]:.3f}", flush=True)

time.sleep(3.0)
p.disconnect(physicsClientId=client)
print("PROBE DONE", flush=True)
