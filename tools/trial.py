"""Building a headless env for one trial (R2, R28).

The course files stay untouched: the point cloud is seeded by wrapping the
sampling function the env calls, and the maze file the env writes goes under
runs/tmp/ and is removed once loaded.
"""
from __future__ import annotations

import contextlib
import functools
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

OBJECT_NAMES = ("master_chef_can sugar_box tomato_soup_can mustard_bottle potted_meat_can banana strawberry "
                "apple bleach_cleanser bowl mug plate large_marker hammer foam_brick").split()


@contextlib.contextmanager
def quiet():
    """The env prints three lines per tick; this drops them."""
    with open(os.devnull, "w") as sink, contextlib.redirect_stdout(sink):
        yield


def make_env(object_idx: int, maze_seed: int, cloud_seed=None, gui: bool = False):
    """A reset env. `cloud_seed` fixes the point-cloud sample; None leaves it
    unseeded, as in the graded runs."""
    import trimesh
    os.chdir(ROOT)                      # the env loads its assets by relative path
    from pick_nav_reach_env_multi import PickNavReachEnv

    folder = ROOT / "runs" / "tmp"
    folder.mkdir(parents=True, exist_ok=True)
    maze = folder / f"maze_{os.getpid()}_{object_idx}_{maze_seed}.urdf"
    sample = trimesh.sample.sample_surface
    if cloud_seed is not None:
        trimesh.sample.sample_surface = functools.partial(sample, seed=int(cloud_seed))
    try:
        with quiet():
            env = PickNavReachEnv(seed=maze_seed, object_idx=object_idx, maze_urdf_path=str(maze), gui=gui)
            env.reset()
    finally:
        trimesh.sample.sample_surface = sample
        maze.unlink(missing_ok=True)
    return env


def close_env(env) -> None:
    import pybullet as p
    if p.isConnected(env.pb_physics_client):
        p.disconnect(env.pb_physics_client)
