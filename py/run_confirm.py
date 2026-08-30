"""CONFIRMATORY RUN. Executes once, per resubmission/CONFIRMATORY_PROTOCOL.md.

500 frozen scenarios x 4 arms = 2000 runs. No retries, no parameter changes.
A scenario failing in any arm is dropped from all four and reported by count.
"""
import glob, json, sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from sfm import Scenario, run_trial

ARMS = [("sfm", "sfm", False), ("sfm-ttc", "sfm", True),
        ("hybrid", "hybrid", False), ("hybrid-ttc", "hybrid", True)]


def one(job):
    path, name, planner, ttc = job
    try:
        S = Scenario.load(path)
        R = run_trial(S, planner, ttc=ttc)
    except Exception as e:                      # recorded, never retried
        return dict(scenario=Path(path).stem, arm=name, failed=True, error=str(e))
    ncol = R.ped_collisions + R.obs_collisions
    return dict(
        scenario=Path(path).stem, condition=S.condition, arm=name, failed=False,
        outcome=R.outcome, goal=int(R.outcome == "success"),
        safe=int(R.outcome == "success" and ncol == 0),
        unsafe_goal=int(R.outcome == "success" and ncol > 0),
        deadlock=int(R.outcome == "deadlock"), timeout=int(R.outcome == "timeout"),
        contacts=ncol, steps=R.steps, path_len=R.path_length,
        min_ped_gap=R.min_ped_gap, min_obs_gap=R.min_obs_gap,
        # mediators, per Amendment A.4b
        ttc_active_frac=R.ttc_active_frac, ttc_activations=R.ttc_activations,
        ttc_brake_time=R.ttc_brake_time,
        first_activation_step=R.first_activation_step,
        sustained_activation_step=R.sustained_activation_step,
        deadlock_onset_step=R.deadlock_onset_step,
        stop_episodes=R.stop_episodes, stop_time=R.stop_time,
        mean_speed=R.mean_speed, min_speed=R.min_speed,
        time_to_goal=R.time_to_goal,
    )


if __name__ == "__main__":
    files = sorted(glob.glob("v2/runs/v2/export_confirm/conf_*.json"))
    jobs = [(f, n, p, t) for f in files for n, p, t in ARMS]
    print(f"CONFIRMATORY RUN: {len(files)} scenarios x 4 arms = {len(jobs)} runs",
          flush=True)
    with Pool(processes=12) as pool:
        res = []
        for i, r in enumerate(pool.imap_unordered(one, jobs), 1):
            res.append(r)
            if i % 100 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)
    Path("v2/runs/v2/confirm_factorial.json").write_text(json.dumps(res, indent=1))
    nf = sum(r.get("failed", False) for r in res)
    print(f"\nwrote confirm_factorial.json  ({len(res)} runs, {nf} failed)")
