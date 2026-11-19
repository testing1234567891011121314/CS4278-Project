"""Per-tick log of the carry (states 5, 67, 68), to find what shakes the grasp.

Usage: python tools/trace_carry.py PROJECT_DIR OBJ SEED MAX_STEPS OUT.csv

Writes one CSV row per env.step while the object is carried: step, state,
navigator status, reach phase, base position and its per-step displacement
(measured and commanded), largest arm-joint velocity, finger forces, bilateral
contact, object drift in the gripper frame (norm, then x toward the fingertips, y along
the closing axis, z across the pads), contacts between the object and anything
but the robot, and the angle between the closing axis and the base's motion. Prints EVENT lines when the grip force on either
finger changes by more than 20 % within 5 steps, and a final RESULT line.
"""
import contextlib, json, os, sys, time
sys.path.insert(0, sys.argv[1]); os.chdir(sys.argv[1]); time.sleep = lambda *a, **k: None
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from pick_nav_reach_env_multi import PickNavReachEnv
from solver import Solver

obj, seed, steps, out = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
quiet = contextlib.redirect_stdout(open(os.devnull, "w"))
with quiet:
    env = PickNavReachEnv(seed=seed, object_idx=obj, maze_urdf_path=f"/tmp/tc{obj}_{seed}_{os.getpid()}.urdf", gui=False)
    env.reset()
    s = Solver(env)
pl, cid = s.grasp_planner, env.pb_physics_client
# Experiments: SOLVER_SET="carry_speed=0.008,tuck_rate=0.006" overrides solver attributes.
for item in filter(None, os.environ.get("SOLVER_SET", "").split(",")):
    key, value = item.split("=")
    assert hasattr(s, key), key
    setattr(s, key, value if isinstance(getattr(s, key), str) else type(getattr(s, key))(float(value)))
f = open(out, "w")
f.write("step,state,nav,phase,bx,by,dbase_mm,dcmd_mm,arm_qvel,f0,f1,bil,drift_mm,other,dx_mm,dy_mm,dz_mm,align_deg,yaw,yphase,galign_deg,dgrip_mm\n")
ref = prev_q = prev_g = None
forces = []
lost = 0
result = dict(object=obj, seed=seed, drop=None)
for t in range(1, steps + 1):
    with quiet:
        a = s.get_action(env)
        obs, info = env.step(a)
    q = np.asarray(obs["qpos"])
    if s.state in (5, 67, 68):
        per = {}
        for c in p.getContactPoints(env.robot_id, env.object_id, physicsClientId=cid):
            if c[3] in pl.finger_ids:
                per[c[3]] = per.get(c[3], 0.0) + c[9]
        f0, f1 = (per.get(k, 0.0) for k in pl.finger_ids)
        bil = f0 > 0.01 and f1 > 0.01
        st = p.getLinkState(env.robot_id, pl.ee, computeForwardKinematics=True, physicsClientId=cid)
        off = Rotation.from_quat(st[5]).inv().apply(np.asarray(obs["object_pos"]) - np.asarray(st[4]))
        ref = off if ref is None else ref
        # Angle between the closing axis (horizontal part) and the base's motion, 0-90 deg.
        closing = Rotation.from_quat(st[5]).as_matrix()[:2, 1]
        move = q[:2] - prev_q[:2] if prev_q is not None else np.zeros(2)
        align = -1.0
        if np.linalg.norm(move) > 1e-5 and np.linalg.norm(closing) > 0.3:
            c = abs(float(closing @ move)) / (np.linalg.norm(closing) * np.linalg.norm(move))
            align = float(np.degrees(np.arccos(min(1.0, c))))
        # The same against the gripper's own motion (differs from the base's when the base turns).
        gpos = np.asarray(st[4])
        gmove = gpos[:2] - prev_g[:2] if prev_g is not None else np.zeros(2)
        galign = -1.0
        if np.linalg.norm(gmove) > 1e-5 and np.linalg.norm(closing) > 0.3:
            c = abs(float(closing @ gmove)) / (np.linalg.norm(closing) * np.linalg.norm(gmove))
            galign = float(np.degrees(np.arccos(min(1.0, c))))
        prev_g = gpos
        yphase = (getattr(s, "_yaw", None) or {}).get("phase", "")
        others = sorted({c[2] for c in p.getContactPoints(bodyA=env.object_id, physicsClientId=cid) if c[2] != env.robot_id})
        dbase = 0.0 if prev_q is None else 1000 * float(np.linalg.norm(q[:2] - prev_q[:2]))
        dcmd = 1000 * float(np.linalg.norm(np.asarray(a)[:2] - (q[:2] if prev_q is None else prev_q[:2])))
        f.write(f"{t},{s.state},{s.curved_nav.status},{getattr(s, '_reach_phase', '')},{q[0]-2:.3f},{q[1]:.3f},"
                f"{dbase:.2f},{dcmd:.2f},{np.max(np.abs(obs['qvel'][3:13])):.3f},{f0:.1f},{f1:.1f},{int(bil)},"
                f"{1000*np.linalg.norm(off-ref):.1f},{'|'.join(map(str, others))},"
                f"{1000*(off-ref)[0]:.1f},{1000*(off-ref)[1]:.1f},{1000*(off-ref)[2]:.1f},{align:.0f},{q[2]:.3f},{yphase},{galign:.0f},{1000*np.linalg.norm(gmove):.2f}\n")
        forces.append((f0, f1))
        if len(forces) > 5:
            old = forces[-6]
            if any(abs(n - o) > 0.2 * max(o, 1.0) for n, o in zip((f0, f1), old)):
                print(f"EVENT step {t} st{s.state} nav={s.curved_nav.status} F {old[0]:.0f}/{old[1]:.0f} -> {f0:.0f}/{f1:.0f} "
                      f"dbase={dbase:.1f}mm base=({q[0]-2:.2f},{q[1]:.2f}) other={others}", flush=True)
        lost = 0 if bil else lost + 1
        if lost == 30 and result["drop"] is None:
            result["drop"] = dict(step=t - 29, state=s.state)
    prev_q = q
    if info["dist_to_goal"] < 0.10 or s.state == 4 or (result["drop"] and lost > 80):
        break
result.update(steps=t, final_state=s.state, goal_dist=round(float(info["dist_to_goal"]), 3), err=getattr(s, "grasp_error", None))
print("RESULT:" + json.dumps(result), flush=True)
