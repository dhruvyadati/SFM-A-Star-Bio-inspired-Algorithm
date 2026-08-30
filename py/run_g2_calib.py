"""Amendment G2 calibration screen: select G_STALL and G_PROG.

    arm A (parameter-free) x 50 + arms B,C x 9 configs x 50 = 950 runs

CALIBRATION ONLY. No result here may be presented as the G2 comparison, and
the 150-scenario evaluation pool is not touched.

The selection rule is implemented below and was frozen in Amendment G2 before
this ran. There is NO "arm A must fail often enough" rule: the manipulation
check is the validity gate.
"""
import json, sys, time
from itertools import product
from multiprocessing import Pool
from pathlib import Path
from collections import defaultdict
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial
from route_recovery import RouteRecovery, RecoveryParams
from planner import GridPlanner

CAL = Path("v2/runs/v2/export_g2_calib")
OUT = Path("v2/runs/v2/g2_calib.json")
STALL = [2.0, 3.0, 4.0]
PROG = [0.10, 0.30, 0.50]
CAL_UNSAFE_TOL = 0.05
WORKERS = 10

def cfgs():
    out = [("A", None)]
    for s, p in product(STALL, PROG):
        out.append((f"B_s{s}_p{p}", (s, p, False)))
        out.append((f"C_s{s}_p{p}", (s, p, True)))
    return out
CONFIGS = cfgs()

def one(job):
    f, label, spec = job
    S = Scenario.load(f)
    tch = json.load(open(f))["mapChange"]["t"]
    rr = None
    if spec is None:
        R = run_trial(S, "hybrid", ttc=False)
    else:
        s, p, rep = spec
        gp = GridPlanner(S.params.xMax, S.params.yMax, S.obstacles,
                         S.params.graphMargin) if rep else None
        rr = RouteRecovery(S.waypoints, S.obstacles, S.params,
                           RecoveryParams(stall_s=s, progress_m=p),
                           replan=rep, planner=gp)
        R = run_trial(S, "hybrid", ttc=False, route_recovery=rr)
    n = R.ped_collisions + R.obs_collisions
    cat = ("safe" if n == 0 else "unsafe") if R.outcome == "success" else (
        "deadlock" if R.outcome == "deadlock" else "other")
    return dict(scenario=Path(f).stem, family=Path(f).stem.rsplit("_", 1)[0],
                config=label, category=cat, steps=int(R.steps),
                path_length=float(R.path_length),
                adv=int(R.route_advancements), replans=int(R.route_replans),
                replans_failed=int(R.route_replans_failed),
                pre_trigger=int(sum(1 for st in (rr.plan_steps if rr else [])
                                    if st * S.params.dt < tch)))

def report(rows):
    by = defaultdict(dict)
    for r in rows: by[r["config"]][r["scenario"]] = r
    sc = sorted(by["A"])
    rate = lambda c, k: float(np.mean([by[c][s]["category"] == k for s in sc]))
    a_unsafe, a_safe = rate("A", "unsafe"), rate("A", "safe")
    print(f"\nARM A: safe {a_safe:.3f}  unsafe {a_unsafe:.3f}  "
          f"deadlock {rate('A','deadlock'):.3f}")
    cands = []
    for s, p in product(STALL, PROG):
        c = f"C_s{s}_p{p}"
        if c not in by: continue
        cands.append(dict(stall=s, prog=p, safe=rate(c, "safe"),
                          d_unsafe=rate(c, "unsafe") - a_unsafe,
                          deadlock=rate(c, "deadlock"),
                          replans=float(np.mean([by[c][x]["replans"] for x in sc])),
                          adv=float(np.mean([by[c][x]["adv"] for x in sc])),
                          pre=int(sum(by[c][x]["pre_trigger"] for x in sc))))
    print(f"\n  {'stall/prog':<12}{'C safe':>9}{'d_unsafe':>10}{'C dead':>9}"
          f"{'replan':>8}{'adv':>7}{'pre':>5}{'pass':>6}")
    for c in sorted(cands, key=lambda r: -r["safe"]):
        c["passes"] = c["d_unsafe"] <= CAL_UNSAFE_TOL
        print(f"  {c['stall']}/{c['prog']:<8}{c['safe']:>9.3f}{c['d_unsafe']:>+10.3f}"
              f"{c['deadlock']:>9.3f}{c['replans']:>8.2f}{c['adv']:>7.2f}"
              f"{c['pre']:>5}{'  yes' if c['passes'] else '   NO':>6}")
    ok = [c for c in cands if c["passes"]]
    if not ok:
        print("\n  NO configuration passed the calibration safety screen.")
        return
    ok.sort(key=lambda r: (-r["safe"], r["replans"], r["adv"], -r["stall"], -r["prog"]))
    w = ok[0]
    print(f"\n  SELECTED by the frozen rule: stall={w['stall']} prog={w['prog']}")
    print(f"    C safe {w['safe']:.3f}  d_unsafe {w['d_unsafe']:+.3f}  "
          f"replans {w['replans']:.2f}  pre-trigger {w['pre']}")

if __name__ == "__main__":
    fs = sorted(str(p) for p in CAL.glob("*.json"))
    jobs = [(f, lab, spec) for f in fs for lab, spec in CONFIGS]
    print(f"scenarios {len(fs)}  configs {len(CONFIGS)}  jobs {len(jobs)}", flush=True)
    t0 = time.time(); rows = []
    with Pool(WORKERS) as pool:
        for i, r in enumerate(pool.imap_unordered(one, jobs, chunksize=4), 1):
            rows.append(r)
            if i % 200 == 0: print(f"  {i}/{len(jobs)}", flush=True)
    wall = time.time() - t0
    OUT.write_text(json.dumps(dict(wall_clock_s=wall, rows=rows)))
    print(f"\nwrote {OUT}  n={len(rows)}  wall {wall/60:.1f} min", flush=True)
    report(rows)
