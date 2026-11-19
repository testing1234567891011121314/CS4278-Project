"""End-to-end headless probe: navigate -> plan (state 1) -> grasp (state 2)
-> carry (state 3). Bypasses the eglRenderer plugin (rendering-only) so
the whole env runs in DIRECT -- no GUI, safe for the laptop, deterministic.
"""
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0  # rendering-only; DIRECT needs no renderer

from pick_nav_reach_env import PickNavReachEnv
from solver import Solver


def main():
    env = PickNavReachEnv(seed=42, gui=False)
    env.reset()
    solver = Solver(env)

    for _ in range(200):
        env.step(np.zeros(env.action_size))

    last_state = None
    info = {}
    for i in range(900):
        action = solver.get_action(env)
        obs, info = env.step(action)
        if solver.state != last_state:
            print(f">>> step {i}: state {last_state} -> {solver.state}",
                  flush=True)
            last_state = solver.state
        if info.get("success"):
            print(f">>> SUCCESS at step {i}", flush=True)
            break

    print("final state:", solver.state,
          "dist_to_goal:", round(float(info.get("dist_to_goal", -1)), 3),
          "success:", info.get("success"))
    stats = getattr(solver, "grasp_last_stats", None)
    if stats:
        print("last ik_stats:", {k: stats.get(k) for k in
                                 ("calls", "ik_fail", "fk_fail", "plan_fail")})


if __name__ == "__main__":
    main()
