"""Trace grasp security from the pick through transport.

Usage: python tools/trace_hold.py PROJECT_DIR OBJ SEED MAX_STEPS [EVERY]

From the moment the solver enters state 2 (grasp execution) it records, per
tick: solver state, bilateral finger contact, finger joint positions and
commands, and the object's drift in the gripper frame relative to where it sat
when the lift finished. Prints a line every EVERY ticks (default 200), on every
state change, and on the first tick contact is lost. Ends with a JSON line
prefixed DROP: (state/tick where contact was lost, or null) and the result.
"""
import sys, os, io, json, time, contextlib
sys.path.insert(0, sys.argv[1]); os.chdir(sys.argv[1]); time.sleep = lambda *a, **k: None
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from pick_nav_reach_env_multi import PickNavReachEnv
from solver import Solver

obj, seed, steps = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
every = int(sys.argv[5]) if len(sys.argv) > 5 else 200
env = PickNavReachEnv(seed=seed, object_idx=obj, maze_urdf_path=f"/tmp/th{obj}_{seed}.urdf", gui=False)
env.reset()
s = Solver(env)
pl = s.grasp_planner
cid = env.pb_physics_client
ee = next(j for j in range(p.getNumJoints(env.robot_id, physicsClientId=cid))
          if p.getJointInfo(env.robot_id, j, physicsClientId=cid)[12].decode() == "gripper_link")


def contact_info():
    cs = p.getContactPoints(env.robot_id, env.object_id, physicsClientId=cid)
    per = {}
    for c in cs:
        if c[3] in pl.finger_ids:
            per[c[3]] = per.get(c[3], 0.0) + c[9]
    return per


def obj_in_gripper(obs):
    st = p.getLinkState(env.robot_id, ee, computeForwardKinematics=True, physicsClientId=cid)
    g, r = np.asarray(st[4]), Rotation.from_quat(st[5])
    return r.inv().apply(np.asarray(obs["object_pos"]) - g)


ref = None
drop = None
history = []
last_state = None
result = dict(object=obj, seed=seed)
for t in range(steps):
    with contextlib.redirect_stdout(io.StringIO()):
        a = s.get_action(env)
        obs, _ = env.step(a)
    gd = float(np.linalg.norm(obs["object_pos"] - obs["goal_pos"]))
    if s.state in (2, 3, 5, 67, 68):
        per = contact_info()
        bil = all(f in per for f in pl.finger_ids)
        off = obj_in_gripper(obs)
        if s.state == 3 and ref is None:
            ref = off.copy()
            cs = [c for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=cid) if c[3] in pl.finger_ids]
            pa = np.mean([c[6] for c in cs if c[3] == pl.finger_ids[0]], axis=0)
            pb = np.mean([c[6] for c in cs if c[3] == pl.finger_ids[1]], axis=0)
            u = (pb - pa) / np.linalg.norm(pb - pa)
            v = np.asarray(obs["object_pos"]) - pa
            result["com_offset"] = round(float(np.linalg.norm(v - v.dot(u) * u)), 4)
            result["axis_z"] = round(float(abs(u[2])), 3)
            print("COM offset from contact line:", result["com_offset"], "axis |z|:", result["axis_z"], flush=True)
        drift = None if ref is None else float(np.linalg.norm(off - ref))
        others = sorted({(c[2], c[4]) for c in p.getContactPoints(bodyA=env.object_id, physicsClientId=cid)
                         if c[2] != env.robot_id})
        q = obs["qpos"]
        line = (f"T{t} st{s.state} bil={bil} F={ {k: round(v, 1) for k, v in per.items()} } "
                f"fq={np.round(obs['qpos'][13:15], 4)} fcmd={np.round(np.asarray(a)[13:15], 4)} "
                f"drift={None if drift is None else round(drift, 3)} obj={np.round(obs['object_pos'], 3)} "
                f"base={np.round([q[0]-2, q[1], q[2]], 3)} other={others} gd={gd:.2f}")
        history.append(line)
        history = history[-40:]
        changed = s.state != last_state
        lost = ref is not None and drop is None and not bil
        if lost:
            drop = dict(t=t, state=s.state, drift=drift, nav=getattr(s.curved_nav, "status", None),
                        phase=getattr(s, "_reach_phase", None),
                        travel_done=getattr(s, "_travel_pose_complete", None))
        if lost:
            print("---- history before loss ----")
            print("\n".join(history[::4]))
            print("---- LOST ----", flush=True)
        elif changed or t % every == 0:
            print(line, flush=True)
        last_state = s.state
    if s.state == 4 or gd < 0.10:
        break
    if drop is not None and t - drop["t"] > 300:
        break
result.update(final_state=s.state, steps=t + 1, goal_dist=gd, err=getattr(s, "grasp_error", None))
print("DROP:" + json.dumps(drop), "RESULT:" + json.dumps(result, default=str), flush=True)
