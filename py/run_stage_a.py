"""Amendment F Stage A: trigger screening on the 160 calibration scenarios.

    2 reference arms + 18 trigger configurations x 160 scenarios = 3200 runs

The ACTION is held fixed at the screening defaults while only the TRIGGER
varies. Screening defaults are NOT selected values; Stage B calibrates the
action on the best two triggers from here.

Calibration set only. Nothing from this file may be presented as the formal
recovery test, and no threshold here is the evaluation margin.

Selection is implemented in `select()` below and is FROZEN before execution:
it is an ordered constraint rule, not "maximise safe success". Ties break
toward the WEAKER intervention, so unnecessary aggressiveness is penalised
rather than rewarded.
"""
from __future__ import annotations

import json
import sys
from itertools import product
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from recovery import RecoveryConfig
from sfm_explore import Scenario, run_trial

FCAL = Path("v2/runs/v2/export_fcal")
OUT = Path("v2/runs/v2/stage_a.json")
WORKERS = 10

# Frozen screening action. NOT selections.
ACTION = dict(relax=0.60, max_recovery=3.0, exit_hold=1.0,
              cooldown=2.0, max_episodes=2)
# Frozen trigger grid: 3 x 3 x 2 = 18
WINDOWS = [2.0, 3.0, 4.0]
PROGRESS = [0.10, 0.20, 0.30]
ACTIVATION = [0.6, 0.8]

# Frozen calibration screens (Amendment F). Screening guardrails only.
CAL_UNSAFE_TOL = 0.050
CAL_RESCUE_FLOOR = -0.150


def configs():
    out = [("reference", None), ("ttc", "TTC_ONLY")]
    for w, p_, a in product(WINDOWS, PROGRESS, ACTIVATION):
        out.append((f"w{w}_p{p_}_a{a}", RecoveryConfig(window=w, theta_prog=p_,
                                                       theta_act=a, **ACTION)))
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
    return dict(scenario=scen, config=label, category=cat, outcome=R.outcome,
                contacts=int(ncol), steps=int(R.steps),
                stop_time=float(R.stop_time), path_length=float(R.path_length),
                ttc_active_frac=float(R.ttc_active_frac),
                min_ped_gap=float(R.min_ped_gap),
                episodes=int(R.recovery_episodes),
                release=float(R.recovery_release_integral),
                rec_steps=int(R.recovery_active_steps),
                suppressed=int(R.recovery_suppressed))


def select(rows):
    """The FROZEN ordered selection rule. Written before Stage A executed."""
    by = {}
    for r in rows:
        by.setdefault(r["config"], {})[r["scenario"]] = r
    scens = sorted(by["ttc"])

    def rate(cfg, pred):
        return float(np.mean([pred(by[cfg][s]) for s in scens]))

    ttc_unsafe = rate("ttc", lambda r: r["category"] == "unsafe")
    # Rescue pool: unsafe WITHOUT the filter, on this calibration set.
    pool = [s for s in scens if by["reference"][s]["category"] == "unsafe"]
    ttc_rescue = (float(np.mean([by["ttc"][s]["category"] == "safe" for s in pool]))
                  if pool else float("nan"))

    surviving = []
    for cfg in by:
        if cfg in ("reference", "ttc"):
            continue
        u = rate(cfg, lambda r: r["category"] == "unsafe")
        resc = (float(np.mean([by[cfg][s]["category"] == "safe" for s in pool]))
                if pool else float("nan"))
        rec = dict(config=cfg, unsafe=u, d_unsafe=u - ttc_unsafe,
                   rescue=resc, d_rescue=resc - ttc_rescue,
                   deadlock=rate(cfg, lambda r: r["category"] == "deadlock"),
                   episodes=float(np.mean([by[cfg][s]["episodes"] for s in scens])),
                   release=float(np.mean([by[cfg][s]["release"] for s in scens])))
        # (1) safety screen  (2) rescue screen
        rec["passes"] = (rec["d_unsafe"] <= CAL_UNSAFE_TOL
                         and (np.isnan(rec["d_rescue"])
                              or rec["d_rescue"] >= CAL_RESCUE_FLOOR))
        surviving.append(rec)

    ok = [r for r in surviving if r["passes"]]
    # (3) lowest deadlock, (4) fewer episodes, (5) shorter release,
    # (6) weaker intervention -- larger window and larger required progress are
    # both LESS eager to fire, so they sort last as tie-breakers.
    def key(r):
        w, p_, a = (float(x[1:]) for x in r["config"].split("_"))
        return (r["deadlock"], r["episodes"], r["release"], -w, -p_, a)
    ok.sort(key=key)
    return dict(ttc_unsafe=ttc_unsafe, ttc_rescue=ttc_rescue, pool=len(pool),
                candidates=sorted(surviving, key=lambda r: r["deadlock"]),
                survivors=len(ok), ranked=ok)


if __name__ == "__main__":
    scens = sorted(p.stem for p in FCAL.glob("fcal_*.json"))
    jobs = [(s, lab, cfg) for s in scens for lab, cfg in CONFIGS]
    print(f"scenarios {len(scens)}  configs {len(CONFIGS)}  jobs {len(jobs)}", flush=True)
    assert len(jobs) == 160 * 20, f"expected 3200, built {len(jobs)}"
    rows = []
    with Pool(WORKERS) as pool_:
        for i, r in enumerate(pool_.imap_unordered(one, jobs, chunksize=4), 1):
            rows.append(r)
            if i % 400 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)
    OUT.write_text(json.dumps(rows))
    print(f"wrote {OUT}  n={len(rows)}", flush=True)
