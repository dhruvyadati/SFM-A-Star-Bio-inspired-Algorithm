"""Dose-response manipulation of intervention strength (Amendment A.5).

Observation cannot establish a pathway; manipulating the proposed mediator can.
If intervention strength drives the trade-off, varying it should move activation,
stationary time, deadlock and safe success in the predicted directions.

"Intervention strength" is not one axis, so each factor is swept ONE AT A TIME
with the others held at their frozen defaults. Varying them jointly could
produce non-monotonic activation even if the mechanism holds.

    ttcHorizon  1.0, 1.5, 2.0, 3.0 s   longer -> activation earlier / broader
    ttcStop     0.4, 0.8, 1.2 s        larger -> full stop demanded sooner
    ttcMinGap   0.20, 0.40, 0.60 m     larger -> separation floor binds more

Both planners are swept. The guided-vs-unguided contrast is prediction 7, the
sharpest test in A.5, and it needs the unguided arm at every level.

STATUS: mechanistic and EXPLORATORY. The factor levels and the Tier 1 / Tier 2
predictions were frozen in Amendment A.5 before any dose-response run; the
sample size and the onset rule are frozen in Amendment D. Nothing here carries
confirmatory status, and no claim in it rests on a significance threshold.

Falsification, as stated in A.5: failure of Tier 1 would falsify the
activation-to-slowing pathway, since those are near-direct consequences of the
control law. Failure of Tier 2 would NOT, by itself -- robot-crowd dynamics are
nonlinear and a stronger early intervention can sometimes resolve a conflict
sooner. Non-monotonic exceptions at the system level do not refute the
mechanism, and must not be reported as if they did.
"""
from __future__ import annotations

import json
import sys
from copy import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial

EXPORT = Path("v2/runs/v2/export_confirm")
OUT = Path("v2/runs/v2/dose_response.json")
WORKERS = 12

DEFAULTS = {"ttcHorizon": 2.0, "ttcStop": 0.8, "ttcMinGap": 0.40}
LEVELS = {
    "ttcHorizon": [1.0, 1.5, 2.0, 3.0],
    "ttcStop": [0.4, 0.8, 1.2],
    "ttcMinGap": [0.20, 0.40, 0.60],
}
PLANNERS = ["hybrid", "sfm"]


def configs():
    """One-factor-at-a-time settings, the shared default emitted once."""
    seen, out = set(), [("reference", None, None, dict(DEFAULTS))]
    for factor, levels in LEVELS.items():
        for lv in levels:
            cfg = dict(DEFAULTS)
            cfg[factor] = lv
            key = tuple(sorted(cfg.items()))
            if key in seen:
                continue
            seen.add(key)
            out.append((factor, lv, f"{factor}={lv}", cfg))
    return out


CONFIGS = configs()


def category(outcome, contacts):
    if outcome == "success":
        return "safe" if contacts == 0 else "unsafe"
    return "deadlock" if outcome == "deadlock" else "other"


def one(job):
    scen, planner, label, cfg, is_ref = job
    S = Scenario.load(str(EXPORT / f"{scen}.json"))
    S.params = replace(S.params, **cfg)
    R = run_trial(S, planner, log=False, ttc=not is_ref)
    # Contact count must match the frozen confirmatory definition in
    # run_confirm.py exactly, or safe/unsafe here means something else.
    ncol = R.ped_collisions + R.obs_collisions
    return dict(
        scenario=scen, planner=planner, config=label,
        category=category(R.outcome, ncol),
        outcome=R.outcome, contacts=int(ncol), steps=int(R.steps),
        stop_time=float(R.stop_time), mean_speed=float(R.mean_speed),
        path_length=float(R.path_length),
        ttc_active_frac=float(R.ttc_active_frac),
        first_activation_step=int(R.first_activation_step),
        sustained_activation_step=int(R.sustained_activation_step),
        mean_speed_active=float(R.mean_speed_active),
        mean_brake_active=float(R.mean_brake_active),
    )


if __name__ == "__main__":
    scens = sorted(p.stem for p in EXPORT.glob("conf_*.json"))
    jobs = []
    for scen in scens:
        for planner in PLANNERS:
            for factor, lv, label, cfg in CONFIGS:
                is_ref = factor == "reference"
                jobs.append((scen, planner, "no-filter" if is_ref else label,
                             cfg, is_ref))

    print(f"scenarios {len(scens)}  planners {len(PLANNERS)}  "
          f"configs {len(CONFIGS)}  jobs {len(jobs)}", flush=True)
    for f, lv, label, _ in CONFIGS:
        print(f"  {'no-filter' if f == 'reference' else label}", flush=True)

    rows = []
    with Pool(WORKERS) as pool:
        for i, r in enumerate(pool.imap_unordered(one, jobs, chunksize=4), 1):
            rows.append(r)
            if i % 200 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)

    OUT.write_text(json.dumps(rows))
    print(f"wrote {OUT}  n={len(rows)}", flush=True)
