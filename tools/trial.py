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


def object_names(library: str = None) -> list[str]:
    """The objects in the order of the env's index: the course's 15, or the
    folders of `library` (a path under the project, e.g. assets/ycb_extra)."""
    if library is None:
        return list(OBJECT_NAMES)
    return [d.name for d in sorted((ROOT / library).glob("*")) if d.is_dir()]


def make_env(object_idx: int, maze_seed: int, cloud_seed=None, gui: bool = False, library: str = None):
    """A reset env. `cloud_seed` fixes the point-cloud sample; None leaves it
    unseeded, as in the graded runs. `library` reads the object from another
    folder of objects in the course's format (see tools/ycb_library.py)."""
    import trimesh
    os.chdir(ROOT)                      # the env loads its assets by relative path
    from pick_nav_reach_env_multi import PickNavReachEnv
    if library is not None:
        folders = [d for d in sorted((ROOT / library).glob("*")) if d.is_dir()]

        class LibraryEnv(PickNavReachEnv):
            """The course env with its object read from another folder. This
            is the env's own `_load_object_table` with only the folder changed."""

            def _load_object_table(self):
                import numpy as np
                import pybullet as p
                import pybullet_data
                p.setAdditionalSearchPath(pybullet_data.getDataPath())
                p.loadURDF("table/table.urdf", baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi / 2]),
                           useFixedBase=1)
                folder = folders[self.object_idx]
                self.object_id = p.loadURDF(str(folder / "coacd_decomposed_object_one_link.urdf"),
                                            basePosition=[0, 0, 1.0], useFixedBase=0)
                self.object_canonical_mesh = trimesh.load(str(folder / "textured.obj"), force="mesh")
                points, faces = trimesh.sample.sample_surface(self.object_canonical_mesh, 1024)
                self.object_canonical_pc = points.astype(np.float32)
                self.object_canonical_normals = self.object_canonical_mesh.face_normals[faces].astype(np.float32)

        PickNavReachEnv = LibraryEnv

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
