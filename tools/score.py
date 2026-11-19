"""Score one headless trial against the rubric in readme.md.

Usage: python tools/score.py PROJECT_DIR OBJ SEED [MAX_STEPS]

"Step" means one env.step call (assumed; see handoff.md). Prints one line,
SCORE:{json}, with:

  lift_step      step at which the solver entered state 3 (grasp + lift done)
  lift10_step    first step the object was >= 10 cm above its resting height
  lift_cm        height above the resting position when state 3 was entered
  hold_min_cm    lowest height during the 50 steps after that
  hold_50        bilateral finger contact on every one of those 50 steps and
                 the object still clear of the table (> 2 cm) at the end
  grasp_pos_err  metres between the planned grasp centre (fixed in the object
  grasp_ang_err  frame at plan time) and the actual one after the 50-step
                 hold; radians between planned and actual gripper orientation
  success_step   first step with the object within 10 cm of the goal
  drop           null, or {step, state}: first step of a run of >= 30 steps
                 without bilateral finger contact after the lift
  stopped        why the trial ended: success / state4 / dropped / stall /
                 max_steps
  points         rubric points for this trial (see POINTS below); max 40

Stops at the first success, 50 steps after a drop, on state 4, on a stall
(no joint moved by > 0.1 mm for 400 steps after the lift) or at MAX_STEPS
(default 15000, the rubric's reach budget).
"""
import contextlib
import json
import os
import sys
import tempfile
import time

project = os.path.abspath(sys.argv[1])
sys.path.insert(0, project)
os.chdir(project)
time.sleep = lambda *a, **k: None  # env.step sleeps 1/240 s per substep

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
from pick_nav_reach_env_multi import PickNavReachEnv
from solver import Solver

LIFT_BUDGET, REACH_BUDGET = 3000, 15000
LIFT_HEIGHT, HOLD_STEPS, GOAL_TOL = 0.10, 50, 0.10
POSE_TOL = 0.10            # the rubric's default tolerance; grasp_pos_err is reported as well
DROP_STEPS, STALL_STEPS = 30, 400

obj, seed = int(sys.argv[2]), int(sys.argv[3])
max_steps = int(sys.argv[4]) if len(sys.argv) > 4 else REACH_BUDGET

quiet = contextlib.redirect_stdout(open(os.devnull, "w"))   # the env and solver print every step
maze = os.path.join(tempfile.gettempdir(), f"score_maze_{os.getpid()}.urdf")
with quiet:
    env = PickNavReachEnv(seed=seed, object_idx=obj, maze_urdf_path=maze, gui=False)
    env.reset()
    solver = Solver(env)
# Experiments: SOLVER_SET="carry_mode=fixed" or "yaw_carry_speed=0.024,..." overrides
# solver attributes (numbers, booleans as 0/1, or strings), as in trace_carry.py.
for item in filter(None, os.environ.get("SOLVER_SET", "").split(",")):
    key, value = item.split("=")
    assert hasattr(solver, key), key
    old = getattr(solver, key)
    setattr(solver, key, value if isinstance(old, str) else type(old)(float(value)))
planner = solver.grasp_planner
cid = env.pb_physics_client
fingers = set(planner.finger_ids)


def bilateral():
    contacts = p.getContactPoints(env.robot_id, env.object_id, physicsClientId=cid)
    return fingers.issubset({c[3] for c in contacts if c[9] > 0.01})


def gripper_pose():
    st = p.getLinkState(env.robot_id, planner.ee, computeForwardKinematics=True, physicsClientId=cid)
    return np.asarray(st[4]), Rotation.from_quat(st[5])


r = dict(object=obj, seed=seed, lift_step=None, lift10_step=None, lift_cm=None, hold_min_cm=None,
         hold_50=False, grasp_pos_err=None, grasp_ang_err=None, success_step=None, drop=None,
         stopped="max_steps")
rest_z = None            # object height when the grasp was planned
planned = None           # (grasp centre in the object frame, gripper orientation in the object frame)
hold_contact = 0
lost = still = 0
last_q = None
states = []
exec_steps = {}       # env steps spent in each phase of the grasp execution (state 2)
t0 = time.perf_counter()
for step in range(1, max_steps + 1):          # step = number of env.step calls made
    with quiet:
        action = solver.get_action(env)
        obs, info = env.step(action)
    state = solver.state
    # The yaw carry's sub-phases (turn, wrist roll, drive) are logged as "67:phase".
    label = state
    if state == 67 and getattr(solver, "_yaw", None):
        label = f"67:{solver._yaw['phase']}"
    if not states or states[-1][1] != label:
        states.append((step, label))
    z = float(obs["object_pos"][2])
    if state == 2 and planned is None:
        plan = solver.grasp_plan
        rest_z = float(plan.object_position[2])
        o_rot = Rotation.from_quat(plan.object_orientation)
        centre, quat = plan.candidate[2], plan.candidate[3]
        planned = (o_rot.inv().apply(centre - plan.object_position), o_rot.inv() * Rotation.from_quat(quat))
        tcp = np.array([planner.last_stats.get("tcp_offset", planner.config.tcp_offset[0]), 0.0, 0.0])
    if state == 2:
        fol = solver.grasp_follower
        ph = fol.plan.phases[fol.index]
        exec_steps[ph] = exec_steps.get(ph, 0) + 1
    if rest_z is not None and r["lift10_step"] is None and z - rest_z >= LIFT_HEIGHT:
        r["lift10_step"] = step
    if state == 3 and r["lift_step"] is None:
        r["lift_step"] = step
        r["lift_cm"] = round(100 * (z - rest_z), 1)
        r["hold_min_cm"] = r["lift_cm"]
    if r["lift_step"] is not None:
        held = bilateral()
        since = step - r["lift_step"]
        if since <= HOLD_STEPS:
            hold_contact += held
            r["hold_min_cm"] = min(r["hold_min_cm"], round(100 * (z - rest_z), 1))
            if since == HOLD_STEPS:
                r["hold_50"] = bool(hold_contact == HOLD_STEPS + 1 and z - rest_z > 0.02)
                g_pos, g_rot = gripper_pose()
                o_rot = Rotation.from_quat(obs["object_xyzw"])
                want = np.asarray(obs["object_pos"]) + o_rot.apply(planned[0])
                r["grasp_pos_err"] = round(float(np.linalg.norm(g_pos + g_rot.apply(tcp) - want)), 4)
                r["grasp_ang_err"] = round(float(((o_rot * planned[1]).inv() * g_rot).magnitude()), 3)
        lost = 0 if held else lost + 1
        if lost == DROP_STEPS and r["drop"] is None:
            r["drop"] = dict(step=step - DROP_STEPS + 1, state=state)
            drop_seen = step
        q = np.asarray(obs["qpos"])
        still = still + 1 if last_q is not None and np.max(np.abs(q - last_q)) < 1e-4 else 0
        last_q = q
    if info["dist_to_goal"] < GOAL_TOL:
        r["success_step"] = step
        r["stopped"] = "success"
        break
    if state == 4:
        r["stopped"] = "state4"
        break
    if r["drop"] is not None and step - drop_seen >= 50:
        r["stopped"] = "dropped"
        break
    if still >= STALL_STEPS:
        r["stopped"] = "stall"
        break

ok = lambda v, limit: v is not None and v <= limit
POINTS = dict(
    lift_3000=6 * ok(r["lift_step"], LIFT_BUDGET),
    lift_10cm=6 * bool(r["hold_min_cm"] is not None and r["hold_min_cm"] >= 100 * LIFT_HEIGHT),
    hold_50=6 * r["hold_50"],
    grasp_pose=6 * bool(r["hold_50"] and ok(r["grasp_pos_err"], POSE_TOL)),
    reach_10cm=6 * (r["success_step"] is not None),
    reach_15000=6 * ok(r["success_step"], REACH_BUDGET),
    no_drop=4 * bool(r["lift_step"] is not None and r["drop"] is None),
)
r.update(
    points={k: int(v) for k, v in POINTS.items()},
    total=int(sum(POINTS.values())),
    steps=step,
    final_state=solver.state,
    goal_dist=round(float(info["dist_to_goal"]), 3),
    err=getattr(solver, "grasp_error", None),
    reach_phase=getattr(solver, "_reach_phase", None),
    nav=getattr(solver.curved_nav, "status", None),
    states=states,
    exec_steps=exec_steps,
    exec_waypoints=({ph: solver.grasp_plan.phases.count(ph) for ph in dict.fromkeys(solver.grasp_plan.phases)}
                    if hasattr(solver, "grasp_plan") else None),
    plan={k: v for k, v in (planner.last_stats or {}).items()
          if k in ("reason", "tried", "candidates", "ik_fail", "transit_fail", "approach_fail", "opening_fail",
                   "closure_fail", "lift_fail", "tcp_attempts", "tcp_offset")},
    wall_s=round(time.perf_counter() - t0, 1),
)
with contextlib.suppress(OSError):
    os.remove(maze)
print("\nSCORE:" + json.dumps(r, default=str), flush=True)
