"""Resumable development screen and locked ORCA robot evaluation."""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
MARGINS = [0.0, 0.02, 0.05, 0.10, 0.20]
PLANNERS = ['hybrid', 'direct']
RULE = 'guided safe rate within 5 percentage points of best; prefer zero total contacts, then smallest margin'
PRIOR_INSPECTION = ['conf_baseline_1012999080', 'conf_baseline_101861417',
                    'conf_baseline_1039889583']


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')
    with temp.open('rb') as stream:
        os.fsync(stream.fileno())
    temp.replace(path)


def sources():
    import numpy
    import rvo2
    return {'python': platform.python_version(), 'numpy': numpy.__version__,
            'rvo2_binary': digest(rvo2.__file__),
            'files': {p.name: digest(p) for p in sorted((ROOT / 'py').glob('*.py'))}}


def key(scenario, planner, margin):
    return f'{scenario}__{planner}__{margin:.2f}'


def trial(job):
    from orca import OrcaConfig, run_trial_orca
    from sfm import Scenario
    path, planner, margin, expected_hash = job
    if digest(path) != expected_hash:
        raise ValueError(f'Scenario changed: {Path(path).name}')
    scenario = Scenario.load(str(path))
    cfg = OrcaConfig(responsibility='ped_margin', responsibility_margin=margin)
    if cfg.radius != scenario.params.rRobot or cfg.max_speed != scenario.params.vMax:
        raise ValueError('Robot radius or speed budget mismatch')
    if planner == 'hybrid' and (scenario.waypoints is None or not scenario.waypoints.size):
        raise ValueError('Guided scenario has no waypoints')
    start = time.monotonic()
    result = run_trial_orca(scenario, mode='orca_robot', cfg=cfg, planner=planner, aligned=True)
    names = ['outcome', 'steps', 'path_length', 'final_dist', 'final_speed',
             'ped_collisions', 'obs_collisions', 'min_ped_gap', 'min_obs_gap']
    row = {name: getattr(result, name) for name in names}
    for name, value in row.items():
        if isinstance(value, float) and not math.isfinite(value):
            if name not in ('min_ped_gap', 'min_obs_gap') or value != math.inf:
                raise ValueError(f'Invalid result {name}: {value}')
            row[name] = None
    row.update(scenario=Path(path).stem, planner=planner, margin=margin,
               scenario_sha256=expected_hash, elapsed_seconds=time.monotonic()-start,
               duration_seconds=result.steps * scenario.params.dt,
               safe=result.outcome == 'success' and result.ped_collisions + result.obs_collisions == 0)
    return row


def summarize(rows):
    summary = []
    for planner, margin in sorted({(r['planner'], r['margin']) for r in rows}):
        group = [r for r in rows if (r['planner'], r['margin']) == (planner, margin)]
        summary.append(dict(planner=planner, margin=margin, n=len(group),
                            safe=sum(r['safe'] for r in group)/len(group),
                            contacts=sum(r['ped_collisions']+r['obs_collisions'] for r in group),
                            deadlock=sum(r['outcome']=='deadlock' for r in group)/len(group)))
    return summary


def select(summary):
    guided = [r for r in summary if r['planner'] == 'hybrid']
    best = max(r['safe'] for r in guided)
    eligible = [r for r in guided if r['safe'] >= best - .05 - 1e-12]
    return min(eligible, key=lambda r: (r['contacts'] > 0, r['margin']))['margin']


def load_rows(output, manifest):
    expected = {key(s, p, m): (s, p, m) for s in manifest['scenarios']
                for p in PLANNERS for m in manifest['margins']}
    rows = {}
    for path in sorted((output/'trials').glob('*.json')):
        row = json.loads(path.read_text())
        k = key(row['scenario'], row['planner'], row['margin'])
        if k not in expected or path.stem != k or k in rows:
            raise ValueError(f'Unexpected or duplicate result: {path.name}')
        if row['scenario_sha256'] != manifest['scenarios'][row['scenario']]:
            raise ValueError(f'Result scenario hash mismatch: {path.name}')
        if row['outcome'] not in ('success', 'deadlock', 'timeout'):
            raise ValueError('Unrecognized outcome')
        if row['safe'] != (row['outcome']=='success' and row['ped_collisions']+row['obs_collisions']==0):
            raise ValueError('Inconsistent safe endpoint')
        rows[k] = row
    return rows, expected


def main():
    from orca import OrcaConfig
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['development', 'freeze', 'evaluation'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--selection', type=Path)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--max-new-trials', type=int)
    args = parser.parse_args()
    if args.workers < 1 or (args.max_new_trials is not None and args.max_new_trials < 1):
        parser.error('Worker and trial limits must be positive')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (output/'.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.stage == 'freeze':
            manifest = json.loads((output/'manifest.json').read_text())
            if manifest['stage'] != 'development' or manifest['runtime'] != sources():
                raise ValueError('Development source/runtime mismatch')
            rows, expected = load_rows(output, manifest)
            if set(rows) != set(expected):
                raise ValueError(f'Incomplete development screen: {len(rows)}/{len(expected)}')
            summary = summarize(list(rows.values()))
            selection = dict(margin=select(summary), rule=RULE, summary=summary,
                             manifest_sha256=digest(output/'manifest.json'),
                             results_sha256={k: digest(output/'trials'/f'{k}.json') for k in sorted(rows)},
                             runtime=manifest['runtime'], prior_inspection=PRIOR_INSPECTION)
            path = output/'selection.json'
            if path.exists() and json.loads(path.read_text()) != selection:
                raise ValueError('Existing selection differs')
            write_json(path, selection)
            print(f'Frozen margin: {selection["margin"]:.2f}', flush=True)
            return
        margins = MARGINS
        selection_hash = None
        if args.stage == 'evaluation':
            if args.selection is None:
                parser.error('Evaluation requires --selection')
            selected = json.loads(args.selection.read_text())
            if selected['runtime'] != sources() or selected['rule'] != RULE:
                raise ValueError('Selection source/runtime or rule mismatch')
            dev = args.selection.parent
            if digest(dev/'manifest.json') != selected['manifest_sha256']:
                raise ValueError('Development manifest changed')
            for k, checksum in selected['results_sha256'].items():
                if digest(dev/'trials'/f'{k}.json') != checksum:
                    raise ValueError('Development results changed')
            dm = json.loads((dev/'manifest.json').read_text())
            dr, expected = load_rows(dev, dm)
            if set(dr) != set(expected) or select(summarize(list(dr.values()))) != selected['margin']:
                raise ValueError('Invalid selection')
            margins = [selected['margin']]
            selection_hash = digest(args.selection)
        pool = 'export_dev' if args.stage == 'development' else 'export_confirm'
        prefix = 'dev_' if args.stage == 'development' else 'conf_'
        paths = sorted((ROOT/'v2/runs/v2'/pool).glob(prefix+'*.json'))
        count = 300 if args.stage == 'development' else 500
        if len(paths) != count:
            raise ValueError(f'Expected {count} scenarios; found {len(paths)}')
        manifest = dict(stage=args.stage, scenarios={p.stem: digest(p) for p in paths},
                        margins=margins, planners=PLANNERS, config=asdict(OrcaConfig()),
                        runtime=sources(), selection_sha256=selection_hash, rule=RULE,
                        prior_inspection=PRIOR_INSPECTION,
                        contact_sampling='pre-update strict negative surface gap; true radii',
                        stepping='shared pre-update state; synchronous integration; post-update waypoint capture')
        mp = output/'manifest.json'
        if mp.exists():
            if json.loads(mp.read_text()) != manifest:
                raise ValueError('Manifest mismatch: use a separate output directory')
        else:
            write_json(mp, manifest)
        (output/'trials').mkdir(exist_ok=True)
        rows, expected = load_rows(output, manifest)
        jobs = [(p, planner, margin, manifest['scenarios'][p.stem]) for p in paths
                for planner in PLANNERS for margin in margins
                if key(p.stem, planner, margin) not in rows]
        if args.max_new_trials:
            jobs = jobs[:args.max_new_trials]
        print(f'{args.stage}: {len(rows)}/{len(expected)} saved; scheduling {len(jobs)}', flush=True)
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = [executor.submit(trial, job) for job in jobs]
            for future in as_completed(futures):
                row = future.result()
                k = key(row['scenario'], row['planner'], row['margin'])
                write_json(output/'trials'/f'{k}.json', row)
                rows[k] = row
                print(f'{len(rows)}/{len(expected)} {k} {row["outcome"]}', flush=True)
        write_json(output/'summary.json', dict(complete=len(rows)==len(expected),
                                             completed=len(rows), expected=len(expected),
                                             groups=summarize(list(rows.values()))))


if __name__ == '__main__':
    main()
