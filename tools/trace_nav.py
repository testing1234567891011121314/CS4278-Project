import sys, time, os
sys.path.insert(0, sys.argv[1]); os.chdir(sys.argv[1]); time.sleep = lambda *a, **k: None
import numpy as np
from pick_nav_reach_env_multi import PickNavReachEnv
from solver import Solver
obj, seed, steps = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
env = PickNavReachEnv(seed=seed, object_idx=obj, maze_urdf_path=f"/tmp/tr{obj}_{seed}.urdf", gui=False); env.reset()
s = Solver(env)
import builtins, io, contextlib
for t in range(steps):
    with contextlib.redirect_stdout(io.StringIO()):
        a = s.get_action(env)
        obs, _ = env.step(a)
    if s.state in (5, 67, 68) and t % 400 == 0:
        q = obs["qpos"]
        print("T", t, "state", s.state, "base", np.round([q[0]-2, q[1]], 2), "obj", np.round(obs["object_pos"], 2),
              "goal_d", round(float(np.linalg.norm(obs["object_pos"]-obs["goal_pos"])), 2),
              "wp", getattr(s.curved_nav, "grid_index", None), "status", getattr(s.curved_nav, "status", None),
              "phase", getattr(s, "_reach_phase", None), flush=True)
    if s.state == 4 or np.linalg.norm(obs["object_pos"]-obs["goal_pos"]) < 0.1:
        print("END", t, s.state, flush=True); break
