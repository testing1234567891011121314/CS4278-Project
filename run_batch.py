"""One fixed maze per batch, one trial per object (optionally repeated)."""
import argparse
import hashlib
import json
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from trial_worker import run_trial


def prepare_maze(batch_dir, seed):
    import numpy as np
    from maze_utils import generate_maze_map, add_left_room_to_maze, create_maze_urdf

    # Dedicated RNG: object surface sampling cannot alter entrance/exit choices.
    rng = np.random.default_rng(seed)
    rows, cols, room = 12, 16, 6
    entrance = (int(rng.integers(1, rows - 1)), 0)
    exit_cell = (int(rng.integers(1, rows - 1)), cols - 1)
    maze = generate_maze_map(
        n_rows=rows, n_cols=cols, in_pos=entrance, out_pos=exit_cell,
        grid_density=0.8, add_x_min_wall=True, add_x_max_wall=True,
        add_y_min_wall=True, add_y_max_wall=True, p_greedy=0.6, seed=seed,
    )
    maze = add_left_room_to_maze(maze, room_width=room)
    maze_path = batch_dir / 'maze.urdf'
    cubes = create_maze_urdf(
        maze, urdf_path=str(maze_path), grid_xy=1.0, grid_z=2.0,
        maze_center_x=(cols-room)/2 + 2.0,
        maze_center_y=0.0, maze_center_z=0.0,
        box_color=(0.95, 0.95, 0.98, 1.0),
    )
    spec = {
        'schema_version': 1, 'maze_seed': seed,
        'urdf_path': str(maze_path),
        'sha256': hashlib.sha256(maze_path.read_bytes()).hexdigest(),
        'entrance': entrance, 'exit': exit_cell,
        'goal_xy': [cols - 0.5 + 2.0, rows/2 - exit_cell[0] - 0.5],
        'cube_positions': np.asarray(cubes).tolist(),
        'parameters': {'rows': rows, 'cols': cols, 'room_width': room,
                       'grid_xy': 1.0, 'grid_z': 2.0, 'grid_density': 0.8,
                       'p_greedy': 0.6},
    }
    spec_path = batch_dir / 'maze.json'
    spec_path.write_text(json.dumps(spec, indent=2), encoding='utf-8')
    return spec_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--max-steps', type=int, default=1000)
    parser.add_argument('--maze-seed', type=int, default=42)
    parser.add_argument('--seed', type=int, default=42,
                        help='Environment seed; shared by all objects in a repeat')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--gpu', type=int, default=0, help='NVENC GPU only')
    parser.add_argument('--objects', type=int, nargs='+', default=list(range(15)))
    args = parser.parse_args()
    if min(args.workers, args.max_steps, args.repeats) < 1:
        parser.error('workers, max-steps and repeats must be positive')
    if args.maze_seed < 0 or args.seed < 0:
        parser.error('seeds must be nonnegative')
    if len(set(args.objects)) != len(args.objects):
        parser.error('object indices must be unique')

    project = Path(__file__).resolve().parent
    # Match the existing environment's sorted glob exactly.
    assets = sorted((project / 'assets' / 'ycb_objects').glob('*'))
    for idx in args.objects:
        if not 0 <= idx < len(assets):
            parser.error(f'Invalid object index {idx}; found {len(assets)} entries')
        for name in ('coacd_decomposed_object_one_link.urdf', 'textured.obj'):
            if not (assets[idx] / name).is_file():
                parser.error(f'Missing object asset: {assets[idx] / name}')

    batch = project / 'runs' / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid4().hex[:8])
    batch.mkdir(parents=True, exist_ok=False)
    maze_spec = prepare_maze(batch, args.maze_seed)
    config = vars(args).copy()
    config['trials'] = len(args.objects) * args.repeats
    config['objects_resolved'] = [{'index': i, 'name': assets[i].name} for i in args.objects]
    (batch / 'config.json').write_text(json.dumps(config, indent=2), encoding='utf-8')
    print(f'Results: {batch}', flush=True)
    failures = 0
    with (batch / 'summary.jsonl').open('w', encoding='utf-8', buffering=1) as summary:
        with ProcessPoolExecutor(max_workers=min(args.workers, config['trials']),
                                 mp_context=mp.get_context('spawn')) as pool:
            futures = {}
            for repeat in range(args.repeats):
                for idx in args.objects:
                    name = f'object_{idx:02d}_{assets[idx].name}_repeat_{repeat:03d}'
                    future = pool.submit(run_trial, str(project), str(batch/name),
                                         args.seed + repeat, args.max_steps, args.gpu,
                                         idx, assets[idx].name, repeat, str(maze_spec))
                    futures[future] = name
            for future in as_completed(futures):
                try:
                    result = future.result()
                except Exception as exc:
                    result = {'trial': futures[future], 'status': 'worker_error', 'error': repr(exc)}
                failures += result['status'] != 'recording_completed'
                summary.write(json.dumps(result) + '\n')
                print(f"{result['trial']}: {result['status']}", flush=True)
    return 1 if failures else 0


if __name__ == '__main__':
    mp.freeze_support()
    raise SystemExit(main())
