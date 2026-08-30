"""Condition-specific transitions and temporal analysis of adverse cases.

POST-OUTCOME EXPLORATORY. Written after the confirmatory results were seen, in
response to the safe->unsafe transitions the transition table revealed. It is
hypothesis-generating and is labelled that way throughout — it is not part of
the frozen protocol and carries none of its evidential status.

Two questions:

1. Condition-specific decomposition, with intervals and discordant counts, so
   the negative `cluttered` point estimate can be examined rather than guessed
   at.

2. What distinguishes the safe->unsafe cases? Three candidate patterns make
   different predictions:

   exposure          activation -> slowing/stopping -> longer local exposure
                     -> contact. Predicts longer activation and stop time than
                     matched safe->safe cases.
   diversion         the filter alters heading or waypoint tracking into a
                     different conflict, without prolonged stopping. Predicts
                     similar stop time but different path length or route.
   pre-intervention  contact occurs before sustained activation. Would weaken
                     the braking explanation for these cases specifically.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

RNG = np.random.default_rng(2718)
CONDS = ["baseline", "dense", "cluttered", "bigObs"]
CATS = ["safe", "unsafe", "deadlock", "other"]


def category(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    if r["outcome"] == "deadlock":
        return "deadlock"
    return "other"


def boot_ci(d, n=4000):
    idx = RNG.integers(0, len(d), size=(n, len(d)))
    m = d[idx].mean(axis=1)
    return float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


res = json.load(open("v2/runs/v2/confirm_factorial.json"))
by, cond = defaultdict(dict), {}
for r in res:
    by[r["scenario"]][r["arm"]] = r
    cond[r["scenario"]] = r["condition"]
complete = [s for s in sorted(by) if len(by[s]) == 4]

print("=" * 74)
print("CONDITION-SPECIFIC TRANSITIONS + TEMPORAL ANALYSIS")
print("POST-OUTCOME EXPLORATORY — hypothesis-generating, not confirmatory")
print("=" * 74)

# --- conditional rate, the right denominator --------------------------
safe_rows = [s for s in complete if category(by[s]["hybrid"]) == "safe"]
to_unsafe = [s for s in safe_rows if category(by[s]["hybrid-ttc"]) == "unsafe"]
to_dead = [s for s in safe_rows if category(by[s]["hybrid-ttc"]) == "deadlock"]
print(f"\nAmong hybrid runs that were SAFE without the filter (n = {len(safe_rows)}):")
print(f"  became unsafe   : {len(to_unsafe):>4}  = {100*len(to_unsafe)/len(safe_rows):.1f}%")
print(f"  became deadlock : {len(to_dead):>4}  = {100*len(to_dead)/len(safe_rows):.1f}%")
print(f"  degraded at all : {len(to_unsafe)+len(to_dead):>4}  "
      f"= {100*(len(to_unsafe)+len(to_dead))/len(safe_rows):.1f}%")
print("  (the marginal 7.4% uses all 500 scenarios as denominator; this")
print("   conditional rate is the risk to a run that was already safe)")

# --- condition-specific ------------------------------------------------
print("\n" + "=" * 74)
print("Per condition: hybrid -> hybrid-ttc")
print("=" * 74)
print(f"{'condition':<11} {'delta':>17} {'disc':>9} {'u>s':>5} {'u>d':>5} "
      f"{'s>u':>5} {'s>d':>5} {'act%':>6} {'stop s':>7}")
print("-" * 74)
for c in CONDS:
    ss = [s for s in complete if cond[s] == c]
    x = np.array([1.0 if category(by[s]["hybrid"]) == "safe" else 0.0 for s in ss])
    y = np.array([1.0 if category(by[s]["hybrid-ttc"]) == "safe" else 0.0 for s in ss])
    d = y - x
    lo, hi = boot_ci(d)
    n01 = int(np.sum((x == 0) & (y == 1)))
    n10 = int(np.sum((x == 1) & (y == 0)))
    tm = defaultdict(int)
    for s in ss:
        tm[(category(by[s]["hybrid"]), category(by[s]["hybrid-ttc"]))] += 1
    act = np.mean([by[s]["hybrid-ttc"]["ttc_active_frac"] for s in ss])
    stp = np.mean([by[s]["hybrid-ttc"]["stop_time"] for s in ss])
    print(f"{c:<11} {d.mean():>+6.3f} [{lo:+.3f},{hi:+.3f}] {n01:>3}/{n10:<5} "
          f"{tm[('unsafe','safe')]:>5} {tm[('unsafe','deadlock')]:>5} "
          f"{tm[('safe','unsafe')]:>5} {tm[('safe','deadlock')]:>5} "
          f"{100*act:>5.1f} {stp:>7.1f}")

# --- temporal comparison across transition groups ----------------------
groups = {
    "safe -> safe":     [s for s in complete if category(by[s]["hybrid"]) == "safe"
                         and category(by[s]["hybrid-ttc"]) == "safe"],
    "safe -> unsafe":   to_unsafe,
    "unsafe -> safe":   [s for s in complete if category(by[s]["hybrid"]) == "unsafe"
                         and category(by[s]["hybrid-ttc"]) == "safe"],
    "unsafe -> deadlock": [s for s in complete if category(by[s]["hybrid"]) == "unsafe"
                           and category(by[s]["hybrid-ttc"]) == "deadlock"],
}

print("\n" + "=" * 74)
print("Temporal / motion profile of the hybrid-ttc run, by transition group")
print("=" * 74)
print(f"{'group':<20} {'n':>4} {'act frac':>9} {'stop s':>8} {'stopEps':>8} "
      f"{'firstAct s':>11} {'meanV':>7} {'pathLen':>8} {'dur s':>7}")
print("-" * 90)
for g, ss in groups.items():
    if not ss:
        continue
    r = [by[s]["hybrid-ttc"] for s in ss]
    fa = [x["first_activation_step"] * 0.01 for x in r if x["first_activation_step"] > 0]
    print(f"{g:<20} {len(ss):>4} "
          f"{np.mean([x['ttc_active_frac'] for x in r]):>9.3f} "
          f"{np.mean([x['stop_time'] for x in r]):>8.2f} "
          f"{np.mean([x['stop_episodes'] for x in r]):>8.1f} "
          f"{(np.mean(fa) if fa else float('nan')):>11.1f} "
          f"{np.mean([x['mean_speed'] for x in r]):>7.3f} "
          f"{np.mean([x['path_len'] for x in r]):>8.1f} "
          f"{np.mean([x['steps'] for x in r]) * 0.01:>7.1f}")

print("\nPaired change relative to the SAME scenario without the filter:")
print(f"{'group':<20} {'d stop s':>10} {'d meanV':>9} {'d pathLen':>10} {'d dur s':>9}")
print("-" * 62)
for g, ss in groups.items():
    if not ss:
        continue
    a = [by[s]["hybrid"] for s in ss]
    b = [by[s]["hybrid-ttc"] for s in ss]
    print(f"{g:<20} "
          f"{np.mean([y['stop_time']-x['stop_time'] for x, y in zip(a,b)]):>+10.2f} "
          f"{np.mean([y['mean_speed']-x['mean_speed'] for x, y in zip(a,b)]):>+9.3f} "
          f"{np.mean([y['path_len']-x['path_len'] for x, y in zip(a,b)]):>+10.1f} "
          f"{np.mean([(y['steps']-x['steps'])*0.01 for x, y in zip(a,b)]):>+9.1f}")

print("\nReading, stated as competing candidates rather than a conclusion:")
print("  exposure    -> safe->unsafe shows MORE stop time and activation than")
print("                 safe->safe, and longer duration in the crowd")
print("  diversion   -> similar stop time but different path length / route")
print("  pre-interv. -> little activation at all in these cases")
