"""Diagnostics bounding the ORCA-responsive result. The study is NOT redesigned.

Four questions, each raised because an aggregate rate could mean two very
different things.

1. RESCUE POOL / CEILING. TTC can only improve safe completion by converting
   unsafe arrivals. If the unfiltered controller is already safe, there is
   nothing to convert and only the mobility cost remains. Measured directly as
   P(unsafe arrival | TTC disabled) per pedestrian model, with the fate of that
   pool decomposed, rather than inferred from aggregate rates.

2. CLEARANCE DEPTH of the marginal failures. Threshold-level grazes and
   substantial collisions produce the same contact count. Replays the affected
   runs and reports how deep the closest approach actually was.

3. RESPONSIBILITY SENSITIVITY beyond safe success. Inflating the robot's
   perceived radius could remove threshold contacts, or it could reshape crowd
   flow entirely. Minimum gap, stop time, activation, deadlock and goal time
   distinguish those.

4. TIMING. Whether failures follow sustained TTC stopping, which is what the
   exposure account requires.

Replays are deterministic, so they reproduce the study's own trajectories.
Nothing here alters the frozen study or its archived outputs.
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict
from copy import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial
from reciprocal import OrcaPedestrians, PedOrcaConfig

EXPORT = Path("v2/runs/v2/export_confirm")
RECIP = Path("v2/runs/v2/reciprocal.json")
CONF = Path("v2/runs/v2/confirm_factorial.json")
WORKERS = 10


def cat(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


def replay(job):
    scen, resp, min_gap, ttc = job
    S = Scenario.load(str(EXPORT / f"{scen}.json"))
    if min_gap is not None:
        S.params = replace(S.params, ttcMinGap=min_gap)
    drv = OrcaPedestrians(S, PedOrcaConfig(responsibility=resp))
    R = run_trial(S, "hybrid", log=False, ttc=ttc, ped_velocity_fn=drv)
    return dict(scenario=scen, responsibility=resp, ttc=bool(ttc),
                min_ped_gap=float(R.min_ped_gap), min_obs_gap=float(R.min_obs_gap),
                contacts=int(R.ped_collisions + R.obs_collisions),
                unsafe_event_step=int(R.unsafe_event_step),
                sustained=int(R.sustained_activation_step),
                first_act=int(R.first_activation_step),
                stop_time=float(R.stop_time), steps=int(R.steps),
                ttc_active_frac=float(R.ttc_active_frac))


def q(v, name, unit=""):
    v = np.asarray([x for x in v if np.isfinite(x)], float)
    if not len(v):
        print(f"    {name:<34} n/a")
        return
    p = np.percentile(v, [10, 25, 50, 75, 90])
    print(f"    {name:<34} n={len(v):>4}  median {p[2]:>7.3f}  "
          f"IQR [{p[1]:>6.3f},{p[3]:>6.3f}]  p10 {p[0]:>7.3f}  min {v.min():>7.3f} {unit}")


def main():
    rows = json.load(open(RECIP))
    by = defaultdict(dict)
    for r in rows:
        by[(r["set"], r["responsibility"], r["arm"])][r["scenario"]] = r
    arch = defaultdict(dict)
    for r in json.load(open(CONF)):
        if not r.get("failed"):
            arch[r["arm"]][r["scenario"]] = r

    print("=" * 88)
    print("DIAGNOSTICS BOUNDING THE ORCA-RESPONSIVE RESULT")
    print("The completed study is unchanged; these characterise it.")
    print("=" * 88)

    # ---- 1. rescue pool -----------------------------------------------------
    print("\n1. RESCUE POOL — what TTC had available to convert")
    print("   TTC raises safe completion only by converting unsafe arrivals.")
    print(f"\n   {'pedestrian model':<38}{'P(unsafe | TTC off)':>21}{'pool n':>9}")
    print("   " + "-" * 68)
    models = [("SFM pedestrians", arch["hybrid"], arch["hybrid-ttc"]),
              ("ORCA-responsive", by[("confirm", "reciprocal", "disabled")],
               by[("confirm", "reciprocal", "ttc0.40")]),
              ("ORCA-resp + inflated radius", by[("confirm", "ped_yield", "disabled")],
               by[("confirm", "ped_yield", "ttc0.40")])]
    for name, off, on in models:
        s = sorted(set(off) & set(on))
        pool = [x for x in s if cat(off[x]) == "unsafe"]
        print(f"   {name:<38}{len(pool)/len(s):>21.3f}{len(pool):>9}")

    print(f"\n   Fate of that pool (rows = model, cols = what the unsafe runs became)")
    print(f"   {'model':<32}{'->safe':>9}{'->deadlock':>12}{'stayed unsafe':>15}")
    print("   " + "-" * 68)
    for name, off, on in models:
        s = sorted(set(off) & set(on))
        pool = [x for x in s if cat(off[x]) == "unsafe"]
        f = defaultdict(int)
        for x in pool:
            f[cat(on[x])] += 1
        print(f"   {name:<32}{f['safe']:>9}{f['deadlock']:>12}{f['unsafe']:>15}")
    print("\n   A small pool caps the achievable benefit regardless of filter quality.")

    # ---- 2 & 3 & 4. replay --------------------------------------------------
    off_r = by[("confirm", "reciprocal", "disabled")]
    on_r = by[("confirm", "reciprocal", "ttc0.40")]
    scens = sorted(off_r)
    marginal = [s for s in scens if cat(off_r[s]) == "safe" and cat(on_r[s]) == "unsafe"]
    control = [s for s in scens if cat(off_r[s]) == "safe" and cat(on_r[s]) == "safe"]
    control = control[:len(marginal)]

    jobs = ([(s, "reciprocal", 0.40, True) for s in marginal]
            + [(s, "ped_yield", 0.40, True) for s in marginal]
            + [(s, "reciprocal", 0.40, True) for s in control]
            + [(s, "reciprocal", None, False) for s in marginal])
    print(f"\n   replaying {len(jobs)} runs for clearance and timing...", flush=True)
    with Pool(WORKERS) as pool:
        out = list(pool.imap_unordered(replay, jobs, chunksize=2))

    lookup = defaultdict(list)
    for r in out:
        lookup[(r["scenario"], r["responsibility"], r["ttc"])].append(r)


    def pick(scen_set, resp, ttc):
        """Select replays by explicit key, never by inferring from a metric.

        An earlier version distinguished TTC-on from TTC-off by activation
        fraction > 0, which silently misclassifies every run where the filter was
        enabled but never fired.
        """
        return [r for s in scen_set for r in lookup.get((s, resp, ttc), [])]


    MARG = pick(marginal, "reciprocal", True)
    MARG_Y = pick(marginal, "ped_yield", True)
    CTRL = pick(control, "reciprocal", True)
    OFF = pick(marginal, "reciprocal", False)

    print("\n2. CLEARANCE DEPTH of the marginal failures")
    print("   min_ped_gap is surface-to-surface; negative = penetration.")
    q([r["min_ped_gap"] for r in MARG], "safe->unsafe, TTC on (standard)", "m")
    q([r["min_ped_gap"] for r in OFF], "same scenarios, TTC off", "m")
    q([r["min_ped_gap"] for r in CTRL], "safe->safe controls, TTC on", "m")
    q([r["min_ped_gap"] for r in MARG_Y], "same scenarios, inflated radius", "m")
    q([r["contacts"] for r in MARG], "contact count per run")

    print("\n3. RESPONSIBILITY SENSITIVITY — does inflation only remove grazes?")
    print(f"   {'quantity':<26}{'standard':>12}{'inflated':>12}{'change':>10}")
    print("   " + "-" * 60)
    for key, lab in (("min_ped_gap", "min ped gap (m)"), ("stop_time", "stop time (s)"),
                     ("ttc_active_frac", "activation"), ("steps", "duration (steps)")):
        a = np.mean([r[key] for r in MARG if np.isfinite(r[key])])
        b = np.mean([r[key] for r in MARG_Y if np.isfinite(r[key])])
        print(f"   {lab:<26}{a:>12.3f}{b:>12.3f}{b-a:>+10.3f}")
    for resp, lab in (("reciprocal", "standard"), ("ped_yield", "inflated")):
        o, n_ = by[("confirm", resp, "disabled")], by[("confirm", resp, "ttc0.40")]
        s = sorted(o)
        dl = np.mean([cat(n_[x]) == "deadlock" for x in s])
        gt = np.mean([n_[x]["steps"] for x in s if cat(n_[x]) in ("safe", "unsafe")]) * 0.01
        print(f"   {'deadlock rate / goal time':<26}{'':>12}{'':>12}"
              f"   {lab}: {dl:.3f} / {gt:.1f}s")

    print("\n4. TIMING — do failures follow sustained TTC intervention?")
    pre = sum(1 for r in MARG if r["unsafe_event_step"] > 0 and 0 < r["sustained"]
              and r["unsafe_event_step"] < r["sustained"])
    post = sum(1 for r in MARG if r["unsafe_event_step"] > 0 and 0 < r["sustained"] <= r["unsafe_event_step"])
    none_ = sum(1 for r in MARG if r["sustained"] <= 0)
    print(f"   contact BEFORE sustained activation : {pre}")
    print(f"   contact AFTER sustained activation  : {post}")
    print(f"   no sustained activation at all      : {none_}")
    print("\n   Contacts before intervention would not be explained by braking.")


if __name__ == "__main__":
    # REQUIRED on macOS: multiprocessing uses spawn, so every child re-imports
    # this module. Without the guard each child re-runs the whole script and
    # creates its own Pool, which deadlocks before a single replay executes.
    main()
