"""Randomised differential test: MATLAB vs Python on unanticipated geometry."""
import csv, glob, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from sfm import Scenario, run_trial

TOL = {"steps_rel": 0.01, "path_rel": 0.001, "final_dist": 1e-3, "gap": 1e-3}
ARMS = {"sfm": ("sfm", False), "sfm-ttc": ("sfm", True)}

mat = {(r["scenario"], r["arm"]): r
       for r in csv.DictReader(open("v2/runs/v2/random_matlab.csv"))}

print("=== Randomised differential test ===\n")
fails, n = [], 0
worst_path = worst_gap = 0.0
for f in sorted(glob.glob("v2/runs/v2/export_rand/rand_*.json")):
    S = Scenario.load(f)
    for arm, (planner, ttc) in ARMS.items():
        m = mat.get((S.name, arm))
        if m is None:
            continue
        R = run_trial(S, planner, ttc=ttc)
        n += 1
        ms, mp = int(float(m["steps"])), float(m["pathLength"])
        mfd, mg = float(m["finalDist"]), float(m["minPedGap"])
        dp = abs(R.path_length - mp)
        dg = 0.0 if (R.min_ped_gap != R.min_ped_gap or mg != mg) else abs(R.min_ped_gap - mg)
        worst_path = max(worst_path, dp)
        if dg == dg:
            worst_gap = max(worst_gap, dg)
        checks = [
            ("outcome", R.outcome == m["outcome"]),
            ("steps", abs(R.steps - ms) <= TOL["steps_rel"] * max(ms, 1)),
            ("path", dp <= TOL["path_rel"] * max(mp, 1e-9)),
            ("final_dist", abs(R.final_dist - mfd) <= TOL["final_dist"]),
        ]
        bad = [k for k, ok in checks if not ok]
        if bad:
            fails.append(f"{S.name}/{arm} ({S.condition}): {', '.join(bad)}")
            print(f"  FAIL {S.name:<10} {arm:<9} {S.condition:<14} "
                  f"py={R.outcome}/{R.steps} mat={m['outcome']}/{ms}")

print(f"{n} runs compared across {len(set(m[0] for m in mat))} randomised scenarios")
print(f"worst path-length difference : {worst_path:.3e} m")
print(f"worst min-gap difference     : {worst_gap:.3e} m")
if fails:
    print(f"\n=== {len(fails)} FAILURE(S) ===")
    for x in fails:
        print("  -", x)
    sys.exit(1)
print("\n=== ALL PASSED — no discrepancy on unanticipated geometry ===")
