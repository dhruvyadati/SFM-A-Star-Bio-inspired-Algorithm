"""Temporal ordering of the safe->unsafe transitions.

POST-OUTCOME EXPLORATORY. Replays the 37 scenarios deterministically with the
exploratory build, which records `unsafe_event_step`. The confirmatory
executable is untouched.

Question: did the unsafe event occur BEFORE meaningful intervention? Higher
activation in adverse cases shows the filter engaged more, but not that it
engaged before the contact. Only the ordering can address that.
"""
import json, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial

def category(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"

res = json.load(open("v2/runs/v2/confirm_factorial.json"))
by, cond = defaultdict(dict), {}
for r in res:
    by[r["scenario"]][r["arm"]] = r; cond[r["scenario"]] = r["condition"]
complete = [s for s in sorted(by) if len(by[s]) == 4]
targets = [s for s in complete if category(by[s]["hybrid"]) == "safe"
           and category(by[s]["hybrid-ttc"]) == "unsafe"]

print(f"Replaying {len(targets)} safe->unsafe scenarios with event timing\n")
buckets = {"before first activation": 0, "after first, before sustained": 0,
           "after sustained activation": 0, "no activation at all": 0}
rows = []
for s in targets:
    S = Scenario.load(f"v2/runs/v2/export_confirm/{s}.json")
    R = run_trial(S, "hybrid", ttc=True)
    u, fa, sa = R.unsafe_event_step, R.first_activation_step, R.sustained_activation_step
    if fa < 0:
        buckets["no activation at all"] += 1; k = "none"
    elif u < fa:
        buckets["before first activation"] += 1; k = "pre-first"
    elif sa < 0 or u < sa:
        buckets["after first, before sustained"] += 1; k = "post-first"
    else:
        buckets["after sustained activation"] += 1; k = "post-sustained"
    rows.append((s, u, fa, sa, k))

print(f"{'scenario':<28} {'unsafe@s':>9} {'firstAct@s':>11} {'sustAct@s':>10}  bucket")
print("-" * 78)
for s, u, fa, sa, k in rows[:12]:
    print(f"{s:<28} {u*0.01:>9.1f} {fa*0.01 if fa>0 else -1:>11.1f} "
          f"{sa*0.01 if sa>0 else -1:>10.1f}  {k}")
if len(rows) > 12:
    print(f"... {len(rows)-12} more")

print(f"\n{'ordering':<34} {'count':>6} {'%':>7}")
print("-" * 50)
for k, v in buckets.items():
    print(f"{k:<34} {v:>6} {100*v/len(targets):>6.1f}%")
print("\nUnsafe events occurring BEFORE first activation would mean the filter")
print("cannot explain those cases. Events after sustained activation are")
print("consistent with braking-induced exposure.")
