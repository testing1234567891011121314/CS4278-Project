"""Where does the wedged marker land when the hand's approach throws it out?
Runs the solver with different arm speeds in transit and prints where the
marker first comes to rest afterwards, and whether the solver then picks it.

    pnr/bin/python tools/experiments/marker_landing.py TRANSIT_STEP [OBJECT=12]
"""
import functools
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trial import close_env, make_env, quiet

import solver as solver_module
from custom_libraries.grasping.executor import ExecutorSettings, GraspExecutor
from solver import Config, Solver, State

step = float(sys.argv[1])
env = make_env(int(sys.argv[2]) if len(sys.argv) > 2 else 12, 42, 300)
solver_module.GraspExecutor = functools.partial(GraspExecutor, settings=ExecutorSettings(transit_step=step))
solver = Solver(env, Config(seed=300))
start, thrown, landed, last, still, done = None, None, None, None, 0, 0
for tick in range(900):
    with quiet():
        obs, _ = env.step(solver.get_action())
    position = np.asarray(obs["object_pos"], dtype=float)
    if tick == 100:
        start = position
    if start is not None and thrown is None and np.linalg.norm(position - start) > 0.05:
        thrown = tick
    if thrown is not None and landed is None:
        still = still + 1 if np.linalg.norm(position - last) < 5e-4 else 0
        if still >= 10:
            landed = (tick, np.round(position, 3))
    last = position
    if solver.state in (State.NAVIGATE, State.STOPPED):
        done += 1
        if done > 5:
            break
lifted = [a.get("lifted_tick") for a in solver.report.attempts] + [solver.attempt.lifted_tick]
print(f"transit step {step}: thrown at tick {thrown}, at rest {landed}, ends {solver.state.value} at tick {solver.tick}, "
      f"lifted at {[t for t in lifted if t]}, failures {[a['failure'] for a in solver.report.attempts][:5]}")
solver.close()
close_env(env)
