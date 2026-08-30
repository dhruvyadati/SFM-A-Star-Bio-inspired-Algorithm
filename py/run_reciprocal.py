"""Amendment E: reciprocal (ORCA) pedestrians, robot controller unchanged.

Does the confirmed exposure mechanism depend on pedestrians that do not yield?

DESIGN. Scenarios are held FIXED and the pedestrian model is the only thing
that changes, so any difference is attributable to the model rather than to
scenario sampling. Running fresh seeds instead would confound the two.

  primary set     the 500 confirmatory scenarios, paired against the archived
                  SFM results scenario by scenario
  disjoint set    the 300 development scenarios, seed-disjoint from the
                  primary set, as a separate check

Arms, frozen: TTC disabled; TTC at ttcMinGap 0.40 (the confirmatory default);
TTC at ttcMinGap 0.20 (the setting the sweep favoured).

Responsibility: 'reciprocal' is standard ORCA. The robot does not run ORCA and
so does not take its half, which means pedestrians supply half the avoidance of
a true ORCA-ORCA pair. 'ped_yield' inflates the robot's radius as pedestrians
see it, increasing clearance without mathematically reassigning responsibility.
Neither is exact full-pedestrian-responsibility and neither is described as
such.
"""
from __future__ import annotations

import json
import sys
from copy import replace
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario
from reciprocal import PedOrcaConfig, run_trial_reciprocal

OUT = Path("v2/runs/v2/reciprocal.json")
WORKERS = 12
ARMS = [("disabled", False, None), ("ttc0.40", True, 0.40), ("ttc0.20", True, 0.20)]
SETS = {"confirm": Path("v2/runs/v2/export_confirm"),
        "dev": Path("v2/runs/v2/export_dev")}


def one(job):
    setname, scen, arm, ttc, min_gap, resp = job
    S = Scenario.load(str(SETS[setname] / f"{scen}.json"))
    if min_gap is not None:
        S.params = replace(S.params, ttcMinGap=min_gap)
    R = run_trial_reciprocal(S, "hybrid", ttc=ttc,
                             cfg=PedOrcaConfig(responsibility=resp))
    ncol = R.ped_collisions + R.obs_collisions      # frozen confirmatory rule
    cat = ("safe" if ncol == 0 else "unsafe") if R.outcome == "success" else (
        "deadlock" if R.outcome == "deadlock" else "other")
    return dict(set=setname, scenario=scen, arm=arm, responsibility=resp,
                category=cat, outcome=R.outcome, contacts=int(ncol),
                steps=int(R.steps), stop_time=float(R.stop_time),
                mean_speed=float(R.mean_speed), path_length=float(R.path_length),
                ttc_active_frac=float(R.ttc_active_frac),
                first_activation_step=int(R.first_activation_step),
                sustained_activation_step=int(R.sustained_activation_step))


if __name__ == "__main__":
    jobs = []
    for setname, d in SETS.items():
        scens = sorted(p.stem for p in d.glob("*.json"))
        for s in scens:
            for arm, ttc, mg in ARMS:
                jobs.append((setname, s, arm, ttc, mg, "reciprocal"))
                # Secondary responsibility mode on the primary set only.
                if setname == "confirm":
                    jobs.append((setname, s, arm, ttc, mg, "ped_yield"))
    print(f"jobs {len(jobs)}", flush=True)
    rows = []
    with Pool(WORKERS) as pool:
        for i, r in enumerate(pool.imap_unordered(one, jobs, chunksize=4), 1):
            rows.append(r)
            if i % 250 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)
    OUT.write_text(json.dumps(rows))
    print(f"wrote {OUT} n={len(rows)}", flush=True)
