"""Four-arm factorial on the 72-scenario development set.

Estimates the paired joint outcome distribution that the confirmatory power
analysis depends on. DEVELOPMENT ONLY -- confirmation seeds stay untouched.

FACTORIAL PURITY. All four arms share identical parameters except the two
factors under study:

    guidance  {none, A*}   x   filter {off, on}

The TTC constants (horizon, stop, clearance, gaps, slew) are the SAME in both
filtered arms, and the social-repulsion gain is nominal everywhere. Nothing is
tuned per arm. Had each arm been separately optimised on development data, the
contrast would estimate a difference between tuned systems rather than the
effect of the factors, which is a different estimand.

Reported per condition, because a single pooled number would hide whether one
stratum drives the whole result:
    interaction estimate, informative fraction, all-fail fraction,
    safe-goal rate by arm, mutually exclusive outcome breakdown.
"""
import glob
import json
import sys
from collections import defaultdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm import Scenario, run_trial

ARMS = [
    ("sfm",        dict(planner="sfm",    ttc=False)),
    ("sfm-ttc",    dict(planner="sfm",    ttc=True)),
    ("hybrid",     dict(planner="hybrid", ttc=False)),
    ("hybrid-ttc", dict(planner="hybrid", ttc=True)),
]


def one(job):
    path, arm_name, kw = job
    S = Scenario.load(path)
    R = run_trial(S, kw["planner"], ttc=kw["ttc"])
    ncol = R.ped_collisions + R.obs_collisions
    return dict(
        scenario=Path(path).stem, condition=S.condition, arm=arm_name,
        outcome=R.outcome, goal=int(R.outcome == "success"),
        safe=int(R.outcome == "success" and ncol == 0),
        deadlock=int(R.outcome == "deadlock"),
        contacts=ncol, steps=R.steps,
        min_ped_gap=R.min_ped_gap, path_len=R.path_length,
    )


if __name__ == "__main__":
    files = sorted(glob.glob("v2/runs/v2/export_dev/dev_*.json"))
    jobs = [(f, n, kw) for f in files for n, kw in ARMS]
    print(f"=== Development factorial: {len(files)} scenarios x {len(ARMS)} arms "
          f"= {len(jobs)} runs ===", flush=True)

    with Pool(processes=12) as pool:
        res = []
        for i, r in enumerate(pool.imap_unordered(one, jobs), 1):
            res.append(r)
            if i % 24 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)

    outname = sys.argv[1] if len(sys.argv) > 1 else "v2/runs/v2/dev_factorial.json"
    with open(outname, "w") as fh:
        json.dump(res, fh, indent=1)

    # --- reshape to per-scenario patterns -----------------------------
    by_scen = defaultdict(dict)
    cond = {}
    for r in res:
        by_scen[r["scenario"]][r["arm"]] = r
        cond[r["scenario"]] = r["condition"]

    order = [a for a, _ in ARMS]
    pats, conds = [], []
    for s, d in sorted(by_scen.items()):
        if len(d) == 4:
            pats.append(tuple(d[a]["safe"] for a in order))
            conds.append(cond[s])
    pats = np.array(pats)
    conds = np.array(conds)

    def delta(p):
        return float(((p[:, 3] - p[:, 2]) - (p[:, 1] - p[:, 0])).mean())

    def contrast(p):
        return (p[:, 3] - p[:, 2]) - (p[:, 1] - p[:, 0])

    print("\n=== Mutually exclusive outcomes by arm (all 72) ===")
    print(f"{'arm':<12} {'safe goal':>10} {'unsafe goal':>12} {'deadlock':>10} "
          f"{'timeout':>9}")
    print("-" * 58)
    for a in order:
        rs = [r for r in res if r["arm"] == a]
        n = len(rs)
        sg = sum(r["safe"] for r in rs)
        ug = sum(r["goal"] - r["safe"] for r in rs)
        dl = sum(r["deadlock"] for r in rs)
        to = n - sg - ug - dl
        print(f"{a:<12} {100*sg/n:>9.1f}% {100*ug/n:>11.1f}% "
              f"{100*dl/n:>9.1f}% {100*to/n:>8.1f}%")

    print("\n=== Per-condition ===")
    print(f"{'condition':<12} {'n':>4} {'delta':>8} {'informative':>12} "
          f"{'all-fail':>9}  safe by arm (sfm/sfm-ttc/hyb/hyb-ttc)")
    print("-" * 96)
    for c in sorted(set(conds)):
        m = conds == c
        p = pats[m]
        d = contrast(p)
        rates = "  ".join(f"{100*p[:, i].mean():5.1f}" for i in range(4))
        print(f"{c:<12} {m.sum():>4} {delta(p):>+8.3f} "
              f"{np.count_nonzero(d)/len(d):>12.2f} "
              f"{(p.sum(axis=1) == 0).mean():>9.2f}  {rates}")

    d_all = contrast(pats)
    print(f"\n{'POOLED':<12} {len(pats):>4} {delta(pats):>+8.3f} "
          f"{np.count_nonzero(d_all)/len(d_all):>12.2f} "
          f"{(pats.sum(axis=1) == 0).mean():>9.2f}")

    print("\nPooling weights the strata by their SAMPLE sizes (equal here), which")
    print("is an estimand choice, not a neutral default. The confirmatory weights")
    print("must be fixed in advance from the intended deployment mixture.")

    np.save("v2/runs/v2/dev_patterns.npy", pats)
    np.save("v2/runs/v2/dev_conditions.npy", conds)
    print(f"\nWrote {outname}, dev_patterns.npy, dev_conditions.npy")
