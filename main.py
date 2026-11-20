"""Run one trial of the solver.

    pnr/bin/python main.py [--object 5] [--seed 42] [--ticks 3000] [--headless]
"""
import argparse
import os
import tempfile

from pick_nav_reach_env_multi import PickNavReachEnv
from solver import Solver


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--object", type=int, default=5, help="index into assets/ycb_objects, 0 to 14")
    parser.add_argument("--seed", type=int, default=42, help="maze seed")
    parser.add_argument("--ticks", type=int, default=3000, help="stop after this many env.step calls")
    parser.add_argument("--headless", action="store_true", help="no window")
    args = parser.parse_args()

    # The env writes the maze it generates to a file; keep that out of assets/.
    maze = os.path.join(tempfile.gettempdir(), f"pnr_maze_{os.getpid()}.urdf")
    env = PickNavReachEnv(seed=args.seed, object_idx=args.object, maze_urdf_path=maze, gui=not args.headless)
    env.reset()
    solver = Solver(env)
    info = env.evaluate()
    for tick in range(args.ticks):
        _, info = env.step(solver.get_action())
        if info["success"]:
            break
    if os.path.exists(maze):
        os.remove(maze)
    print(f"\nticks: {tick + 1}  state: {solver.state.value}  delivered: {bool(info['success'])}  "
          f"distance to goal: {info['dist_to_goal']:.3f} m  errors: {len(solver.report.errors)}")
    return 0 if info["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
