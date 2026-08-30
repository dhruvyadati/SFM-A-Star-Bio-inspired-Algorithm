"""Amendment F Stage B: action calibration on the 160 calibration scenarios.

    2 triggers x 3 release x 3 duration x 3 cap = 54 configurations
    + 2 reference arms, x 160 scenarios = 8960 runs

Frozen before execution (Amendment F, 6 Aug 2026). The grid is NOT refined
after Stage A: if no candidate approaches the target, the conclusion is that
the architecture lacks leverage, not that the grid needs more levels.

TRIGGER 2 IS A NEAR-DUPLICATE. It was selected by the ranking frozen before
Stage A, not for diversity, and differs from Trigger 1 in 1 of 160 scenarios.
Agreement between them is therefore guaranteed by construction and is NOT
evidence of robustness -- `report()` states this rather than presenting the
agreement as support.

FUTILITY RULE, frozen before this ran: do not launch the formal evaluation if
no candidate reduces calibration deadlock by at least 3 percentage points while
staying inside the calibration screens.
"""
from __future__ import annotations

import json
import sys
import time
from itertools import product
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from recovery import RecoveryConfig
from sfm_explore import Scenario, run_trial

FCAL = Path("v2/runs/v2/export_fcal")
OUT = Path("v2/runs/v2/stage_b.json")
WORKERS = 10

TRIGGERS = {                      # frozen ranking from Stage A
    "T1": dict(window=4.0, theta_prog=0.30, theta_act=0.80),
    "T2": dict(window=4.0, theta_prog=0.20, theta_act=0.80),
}
RELEASE = [0.4, 0.6, 0.8]
DURATION = [2.0, 3.0, 5.0]
CAP = [1, 2, 3]
FIXED = dict(exit_hold=1.0, cooldown=2.0)     # structural, not calibrated

CAL_UNSAFE_TOL = 0.050
CAL_RESCUE_FLOOR = -0.150
FUTILITY_DEADLOCK = 0.030         # frozen before execution


def configs():
    out = [("reference", None), ("ttc", "TTC_ONLY")]
    for tk, rel, dur, cap in product(TRIGGERS, RELEASE, DURATION, CAP):
        out.append((f"{tk}_r{rel}_d{dur}_c{cap}",
                    RecoveryConfig(relax=rel, max_recovery=dur, max_episodes=cap,
                                   **TRIGGERS[tk], **FIXED)))
    return out


CONFIGS = configs()


def one(job):
    scen, label, cfg = job
    S = Scenario.load(str(FCAL / f"{scen}.json"))
    if label == "reference":
        R = run_trial(S, "hybrid", ttc=False)
    elif cfg == "TTC_ONLY":
        R = run_trial(S, "hybrid", ttc=True)
    else:
        R = run_trial(S, "hybrid", ttc=True, recovery_cfg=cfg)
    ncol = R.ped_collisions + R.obs_collisions
    cat = ("safe" if ncol == 0 else "unsafe") if R.outcome == "success" else (
        "deadlock" if R.outcome == "deadlock" else "other")
    return dict(scenario=scen, config=label, category=cat, steps=int(R.steps),
                stop_time=float(R.stop_time), min_ped_gap=float(R.min_ped_gap),
                episodes=int(R.recovery_episodes),
                release=float(R.recovery_release_integral),
                rec_steps=int(R.recovery_active_steps),
                exhausted=bool(R.recovery_exhausted))


def report(rows):
    by = {}
    for r in rows:
        by.setdefault(r["config"], {})[r["scenario"]] = r
    scens = sorted(by["ttc"])
    rate = lambda c, k: float(np.mean([by[c][s]["category"] == k for s in scens]))
    pool = [s for s in scens if by["reference"][s]["category"] == "unsafe"]
    resc = lambda c: (float(np.mean([by[c][s]["category"] == "safe" for s in pool]))
                      if pool else float("nan"))
    ttc_u, ttc_d, ttc_r = rate("ttc", "unsafe"), rate("ttc", "deadlock"), resc("ttc")

    # --- reporting rule 2: are the action parameters non-inert? ------------
    print("\nNON-INERTNESS OF ACTION PARAMETERS")
    def sig(c):
        return tuple((by[c][s]["steps"], by[c][s]["category"],
                      by[c][s]["episodes"], round(by[c][s]["release"], 9))
                     for s in scens)
    base = "T1_r0.6_d3.0_c2"
    for name, alts in (("release", [f"T1_r{x}_d3.0_c2" for x in RELEASE]),
                       ("duration", [f"T1_r0.6_d{x}_c2" for x in DURATION]),
                       ("cap", [f"T1_r0.6_d3.0_c{x}" for x in CAP])):
        n = len({sig(a) for a in alts if a in by})
        print(f"  {name:<10} distinct behaviours across levels: {n}/{len(alts)}"
              + ("" if n > 1 else "   <-- INERT, ranking is meaningless"))

    # --- candidate table ---------------------------------------------------
    cands = []
    for c in by:
        if c in ("reference", "ttc"):
            continue
        d_unsafe = rate(c, "unsafe") - ttc_u
        d_resc = resc(c) - ttc_r
        cands.append(dict(config=c, d_deadlock=rate(c, "deadlock") - ttc_d,
                          d_unsafe=d_unsafe, d_rescue=d_resc,
                          safe=rate(c, "safe"),
                          episodes=float(np.mean([by[c][s]["episodes"] for s in scens])),
                          release=float(np.mean([by[c][s]["release"] for s in scens])),
                          passes=(d_unsafe <= CAL_UNSAFE_TOL
                                  and (np.isnan(d_resc) or d_resc >= CAL_RESCUE_FLOOR))))
    cands.sort(key=lambda r: (r["d_deadlock"], r["episodes"], r["release"]))
    print(f"\nTTC reference: unsafe {ttc_u:.3f}  deadlock {ttc_d:.3f}  rescue {ttc_r:.3f}")
    print(f"\n  {'config':<20}{'d_deadlock':>12}{'d_unsafe':>10}{'d_rescue':>10}{'eps':>7}{'pass':>6}")
    for c in cands[:12]:
        print(f"  {c['config']:<20}{c['d_deadlock']:>+12.3f}{c['d_unsafe']:>+10.3f}"
              f"{c['d_rescue']:>+10.3f}{c['episodes']:>7.2f}"
              f"{'  yes' if c['passes'] else '   NO':>6}")

    # --- reporting rule 1: trigger sensitivity ----------------------------
    print("\nTRIGGER SENSITIVITY (T1 vs T2, same action)")
    diffs = []
    for rel, dur, cap in product(RELEASE, DURATION, CAP):
        a, b = f"T1_r{rel}_d{dur}_c{cap}", f"T2_r{rel}_d{dur}_c{cap}"
        if a in by and b in by:
            diffs.append(abs(rate(a, "deadlock") - rate(b, "deadlock")))
    print(f"  mean |d_deadlock(T1) - d_deadlock(T2)| over 27 actions: {np.mean(diffs):.4f}")
    print("  NOTE: T1 and T2 differ in 1/160 scenarios by construction, so")
    print("  agreement here is NOT evidence of robustness. The frozen ranking")
    print("  produced a near-duplicate; the intended robustness stratum could")
    print("  not be exercised.")

    # --- futility rule -----------------------------------------------------
    ok = [c for c in cands if c["passes"] and c["d_deadlock"] <= -FUTILITY_DEADLOCK]
    print(f"\nFUTILITY RULE (frozen before execution): need d_deadlock <= "
          f"-{FUTILITY_DEADLOCK:.3f} within the screens")
    if ok:
        w = ok[0]
        print(f"  PASSED — {len(ok)} candidate(s). Best: {w['config']} "
              f"d_deadlock {w['d_deadlock']:+.3f}, d_unsafe {w['d_unsafe']:+.3f}")
    else:
        best = min((c for c in cands if c["passes"]),
                   key=lambda r: r["d_deadlock"], default=None)
        print(f"  NOT PASSED. Best passing candidate: "
              f"{best['config'] if best else 'none'} "
              f"d_deadlock {best['d_deadlock']:+.3f}" if best else "  none")
        print("  -> Do NOT launch the formal evaluation. Report that a")
        print("     progress-aware brake release did not deliver a worthwhile")
        print("     deadlock reduction on the calibration set.")
    return cands


if __name__ == "__main__":
    scens = sorted(p.stem for p in FCAL.glob("fcal_*.json"))
    jobs = [(s, lab, cfg) for s in scens for lab, cfg in CONFIGS]
    print(f"scenarios {len(scens)}  configs {len(CONFIGS)}  jobs {len(jobs)}", flush=True)
    assert len(CONFIGS) == 56, f"expected 54 + 2 reference, got {len(CONFIGS)}"
    t0 = time.time()
    rows = []
    with Pool(WORKERS) as pool_:
        for i, r in enumerate(pool_.imap_unordered(one, jobs, chunksize=4), 1):
            rows.append(r)
            if i % 800 == 0:
                el = time.time() - t0
                print(f"  {i}/{len(jobs)}  {el/60:.0f} min elapsed, "
                      f"~{el/i*(len(jobs)-i)/60:.0f} min left", flush=True)
    wall = time.time() - t0
    OUT.write_text(json.dumps(dict(wall_clock_s=wall, workers=WORKERS,
                                   n_runs=len(rows), rows=rows)))
    print(f"\nwrote {OUT}  n={len(rows)}  wall clock {wall/3600:.2f} h", flush=True)
    report(rows)
