"""Reviewer-item closure runs: 1.11, 1.34, 1.37.

FROZEN PROTOCOL: resubmission/REVIEWER_CLOSURE_PROTOCOL.md, written before this
ran. Executed once. No item is re-run with different settings after results are
seen. 1.13 is closed separately by analyze_rgoal_sensitivity.py with no runs.

All scenarios come from the 500 frozen confirmatory maps. None is generated.
The frozen route_recovery.py is NOT modified: PeriodicReplanner below exposes
the same update(step, x, wi) hook the engine already calls.
"""
import json, sys, time
from dataclasses import replace
from pathlib import Path
from multiprocessing import Pool
from collections import defaultdict
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial
from planner import GridPlanner, prune_collinear

SRC = Path("v2/runs/v2/export_confirm")
OUT = Path("v2/runs/v2/reviewer_closure.json")
WORKERS = 10

# ---- 1.34 frozen factors: one at a time around nominal, NOT a Cartesian sweep
NOMINAL = dict(tau=0.50, v0Robot=1.30, aPed=2.1, aObs=10.0,
               rInfPed=5.0, rWaypoint=2.0, dt=0.01)
FACTORS = {
    "tau":       (0.35, 0.70),
    "v0Robot":   (1.0, 1.6),
    "aPed":      (1.5, 3.0),
    "aObs":      (6.0, 15.0),
    "rInfPed":   (3.0, 7.0),
    "rWaypoint": (1.0, 3.0),
    "dt":        (0.005, 0.02),
}
PERIOD_S = 2.0          # 1.11, frozen single value
SUBSET_PER_COND = 25    # 1.34, frozen lexical rule


class PeriodicReplanner:
    """1.11 baseline: replan every PERIOD_S seconds, unconditionally.

    Deliberately NOT progress-triggered -- that is Amendment G2 arm C, which is
    a different controller. This fires on a clock and nothing else.
    """

    def __init__(self, P, obstacles, period_s):
        self.P = P
        self.planner = GridPlanner(P.xMax, P.yMax, obstacles, P.graphMargin)
        n = period_s / P.dt
        assert abs(n - round(n)) < 1e-9, "period must be a whole number of steps"
        self.every = int(round(n))
        self.n_plan = 0
        self.n_plan_failed = 0
        self.plan_steps = []
        # The engine reads n_adv off any route object. This baseline never
        # advances waypoints -- it only replans -- so it is structurally zero.
        self.n_adv = 0
        self.adv_steps = []

    def update(self, step, x, wi):
        if step == 0 or step % self.every:
            return None, None
        path = self.planner.plan((x[0], x[1]), self.P.goal)
        if path is None or path.shape[1] < 2:
            self.n_plan_failed += 1
            return None, None
        self.n_plan += 1
        self.plan_steps.append(step)
        return prune_collinear(path), 0


def _scale_dt(P, dt):
    """Hold physical durations fixed when dt changes (protocol 1.34)."""
    k = NOMINAL["dt"] / dt
    return replace(P, dt=dt,
                   maxSteps=int(round(P.maxSteps * k)),
                   dwellSteps=int(round(P.dwellSteps * k)),
                   stallSteps=int(round(P.stallSteps * k)))


def one(job):
    path, item, label, spec = job
    S = Scenario.load(path)
    rr = None

    if item == "1.11" and label == "hybrid-periodic":
        rr = PeriodicReplanner(S.params, S.obstacles, PERIOD_S)

    elif item == "1.37" and label == "hybrid-nopruning":
        gp = GridPlanner(S.params.xMax, S.params.yMax, S.obstacles,
                         S.params.graphMargin)
        raw = gp.plan(S.params.start, S.params.goal)
        if raw is None:
            return dict(scenario=S.name, condition=S.condition, item=item,
                        config=label, failed=True)
        S = replace(S, waypoints=raw)          # raw A* path, pruning ablated

    elif item == "1.34" and spec is not None:
        fac, val = spec
        P = _scale_dt(S.params, val) if fac == "dt" else replace(S.params, **{fac: val})
        S = replace(S, params=P)

    R = run_trial(S, "hybrid", ttc=False, route_recovery=rr)
    n = R.ped_collisions + R.obs_collisions
    return dict(
        scenario=S.name, condition=S.condition, item=item, config=label,
        failed=False, outcome=R.outcome,
        goal=int(R.outcome == "success"),
        safe=int(R.outcome == "success" and n == 0),
        unsafe_goal=int(R.outcome == "success" and n > 0),
        deadlock=int(R.outcome == "deadlock"),
        timeout=int(R.outcome == "timeout"),
        contacts=int(n), steps=int(R.steps), path_len=float(R.path_length),
        min_obs_gap=float(R.min_obs_gap), min_ped_gap=float(R.min_ped_gap),
        replans=int(rr.n_plan if rr else 0),
        replans_failed=int(rr.n_plan_failed if rr else 0))


def build_jobs():
    files = sorted(str(p) for p in SRC.glob("*.json"))
    assert len(files) == 500, "expected the 500 frozen confirmatory scenarios"

    # 1.34 subset: first 25 per condition by sorted filename. Lexical, no outcome.
    per = defaultdict(list)
    for f in files:
        per[Path(f).stem.split("_")[1]].append(f)
    subset = [f for c in sorted(per) for f in per[c][:SUBSET_PER_COND]]
    assert len(subset) == 100, f"subset is {len(subset)}, expected 100"

    jobs = []
    for f in files:
        jobs.append((f, "1.11", "hybrid-periodic", None))
        jobs.append((f, "1.37", "hybrid-nopruning", None))
    for f in subset:
        jobs.append((f, "1.34", "nominal", None))
        for fac, (lo, hi) in FACTORS.items():
            jobs.append((f, "1.34", f"{fac}={lo}", (fac, lo)))
            jobs.append((f, "1.34", f"{fac}={hi}", (fac, hi)))
    return jobs, subset


if __name__ == "__main__":
    jobs, subset = build_jobs()
    print(f"jobs {len(jobs)}  (1.11: 500, 1.37: 500, 1.34: {15*100})", flush=True)
    t0 = time.time(); rows = []
    with Pool(WORKERS) as pool:
        for i, r in enumerate(pool.imap_unordered(one, jobs, chunksize=2), 1):
            rows.append(r)
            if i % 100 == 0:
                el = time.time() - t0
                print(f"  {i}/{len(jobs)}  {el/60:.1f} min elapsed, "
                      f"~{(el/i)*(len(jobs)-i)/60:.0f} min left", flush=True)
    wall = time.time() - t0
    OUT.write_text(json.dumps(dict(
        wall_clock_s=wall, period_s=PERIOD_S, nominal=NOMINAL,
        factors={k: list(v) for k, v in FACTORS.items()},
        subset=[Path(x).stem for x in subset], rows=rows)))
    print(f"\nwrote {OUT}  n={len(rows)}  wall {wall/60:.1f} min", flush=True)
