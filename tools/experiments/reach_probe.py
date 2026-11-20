"""Why does the final reach fail for an object picked from the floor? Runs a
trial until the solver reports the goal out of reach, then tries the same
straight line by hand and prints what stops it.

    pnr/bin/python tools/experiments/reach_probe.py [OBJECT=10]
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trial import close_env, make_env, quiet

from custom_libraries.common import Deadline
from custom_libraries.grasping import ik
from solver import Config, Solver, State

env = make_env(int(sys.argv[1]) if len(sys.argv) > 1 else 10, 42, 300)
solver = Solver(env, Config(seed=300))
found = False
for tick in range(3000):
    before = solver._hold.copy()
    with quiet():
        obs, info = env.step(solver.get_action())
    failure = solver.attempt.failure or ""
    if solver.state is State.RETRY and "out of reach" in failure:
        found = True
        break
    if solver.state in (State.DONE, State.STOPPED):
        break
print("stopped at tick", solver.tick, "state", solver.state.value, "failure", solver.attempt.failure)
if found:
    scene, robot = solver.scene, solver.scene.robot
    q = before
    position, turn = scene.sim.gripper_pose(q)
    gap = scene.world.goal - np.asarray(obs["object_pos"], dtype=float)
    joints = list(robot.plan_joints)
    print("object", np.round(obs["object_pos"], 3), "goal", scene.world.goal, "gap", np.round(gap, 3))
    print("hand at", np.round(position, 3), "pointing", np.round(turn[:, 0], 2), "base", np.round(robot.base_pose(q), 3))
    print("torso and arm:", np.round(q[joints], 3))
    print("room below / above:", np.round(q[joints] - robot.lower[joints], 3), np.round(robot.upper[joints] - q[joints], 3))
    measured = np.asarray(obs["qpos"], dtype=float)
    print("commanded minus measured:", np.round((q - measured)[joints], 4))
    for label, restarts, iterations in (("no restart, 80 iterations", 0, 80), ("no restart, 400", 0, 400), ("6 restarts", 6, 80)):
        out = ik.solve(scene.sim, position + 0.01 * gap / np.linalg.norm(gap), turn, q, np.random.default_rng(0),
                       Deadline(5.0), restarts=restarts, iterations=iterations)
        print(f"1 cm toward the goal, {label}:", None if out is None else np.round(out[joints] - q[joints], 3))
    j = ik.jacobian(scene.sim, q)
    print("singular values of the hand's Jacobian:", np.round(np.linalg.svd(j, compute_uv=False), 4))
solver.close()
close_env(env)
