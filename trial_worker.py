"""Process-local trial execution; all trials load the batch's immutable maze."""
import json
import traceback
from pathlib import Path


def json_value(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'Cannot serialize {type(value)}')


def run_trial(project_dir, trial_dir, seed, max_steps, gpu,
              object_idx, object_name, repeat_id, maze_spec_path):
              # top of trial_worker.py, before `import pybullet as p`
    import os
    os.environ.setdefault("DISPLAY", ":0")
    os.environ.setdefault("EGL_PLATFORM", "x11")
    import contextlib
    import hashlib
    import os
    import time
    import numpy as np
    import pybullet as p
    from pick_nav_reach_env import PickNavReachEnv
    from solver import Solver
    from custom_libraries.recording import record_navigation

    project = Path(project_dir).resolve()
    trial = Path(trial_dir).resolve()
    trial.mkdir(parents=True, exist_ok=False)
    previous_cwd = Path.cwd()
    started = time.perf_counter()
    env = None
    result = dict(trial=trial.name, seed=seed, object_idx=object_idx,
                  object_name=object_name, repeat_id=repeat_id, gpu=gpu,
                  max_steps=max_steps, directory=str(trial),
                  task_success=None, navigation_success=None, status='starting')
    try:
        with (trial / 'worker.log').open('w', encoding='utf-8', buffering=1) as log:
            with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                try:
                    os.chdir(project)
                    spec = json.loads(Path(maze_spec_path).read_text(encoding='utf-8'))
                    maze_path = Path(spec['urdf_path'])
                    if hashlib.sha256(maze_path.read_bytes()).hexdigest() != spec['sha256']:
                        raise RuntimeError('Batch maze checksum mismatch')
                    result.update(maze_seed=spec['maze_seed'], maze_sha256=spec['sha256'],
                                  maze_path=str(maze_path))
                    object_path = sorted((project/'assets'/'ycb_objects').glob('*'))[object_idx]
                    if object_path.name != object_name:
                        raise RuntimeError('Object index mapping changed since batch setup')

                    # A batch-only override prevents the original constructor from
                    # regenerating the maze after object-dependent random sampling.
                    class FixedMazeEnv(PickNavReachEnv):
                        def _load_maze(self):
                            self.maze_out_pos_x, self.maze_out_pos_y = spec['goal_xy']
                            self.cube_positions = np.asarray(spec['cube_positions'], dtype=float)
                            p.loadURDF(str(maze_path), useFixedBase=True,
                                       flags=p.URDF_MERGE_FIXED_LINKS,
                                       physicsClientId=self.pb_physics_client)

                    env = FixedMazeEnv(seed=seed, object_idx=object_idx,
                                       maze_urdf_path=maze_path, gui=False)
                    env.reset()
                    solver = Solver(env)
                    p.resetDebugVisualizerCamera(
                        cameraDistance=8.0, cameraYaw=45, cameraPitch=-60,
                        cameraTargetPosition=[0, 0, 0], physicsClientId=env.pb_physics_client)
                    os.chdir(trial)
                    try:
                        record_navigation(
                            env, solver,
                            encoder_args=['-c:v', 'h264_nvenc', '-gpu', str(gpu),
                                          '-preset', 'p1', '-rc', 'constqp', '-qp', '23'],
                            label=f'object_{object_idx:02d}_seed_{seed}',
                            max_steps=max_steps, interactive=False)
                    finally:
                        os.chdir(project)
                    info = env.evaluate()
                    nav_status = getattr(solver.curved_nav, 'status', 'unknown')
                    result.update(status='recording_completed',
                                  task_success=bool(info.get('success', False)),
                                  navigation_success=(nav_status == 'reached'),
                                  navigation_status=nav_status, final_info=info,
                                  waypoint_index=getattr(solver, 'waypoint_index', None),
                                  path_length=len(solver.path) if solver.path is not None else None)
                except Exception as exc:
                    result.update(status='error', error=str(exc), traceback=traceback.format_exc())
                    traceback.print_exc()
                finally:
                    try:
                        # One connection per worker; also clean up partially failed constructors.
                        client = env.pb_physics_client if env is not None else 0
                        if p.isConnected(client):
                            p.disconnect(client)
                    except Exception:
                        result['cleanup_error'] = traceback.format_exc()
                    result['wall_seconds'] = time.perf_counter() - started
                    result = json.loads(json.dumps(result, default=json_value))
                    (trial/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    finally:
        os.chdir(previous_cwd)
    return result
