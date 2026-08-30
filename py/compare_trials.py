"""Cross-validate the REAL trial configuration: many pedestrians AND obstacles.

The canonical fixtures carry at most 5 pedestrians and one obstacle. Every
production run has 48 pedestrians and ~10 obstacles simultaneously, and that
combined path has never been compared against MATLAB. Two earlier bugs
appeared only in code paths no fixture exercised, so this gap has to close
before the 72-scenario development result is anything but provisional.

All four factorial arms are checked, not just the unfiltered one.
"""
import csv
import glob
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from sfm import Scenario, run_trial

# Same prespecified tolerances as the canonical comparison. Not loosened for
# the harder configuration -- if the engines diverge here, that is the result.
TOL = {"steps_rel": 0.01, "path_rel": 0.001, "final_dist": 1e-3,
       "min_gap": 1e-3}

ARMS = {"sfm": ("sfm", False), "sfm-ttc": ("sfm", True),
        "hybrid": ("hybrid", False), "hybrid-ttc": ("hybrid", True)}

mat = {}
with open("v2/runs/v2/trial_matlab.csv") as f:
    for r in csv.DictReader(f):
        mat[(r["scenario"], r["arm"])] = r

print("=== Python vs MATLAB: 48 pedestrians + 10 obstacles, all four arms ===\n")
print(f"{'scenario':<18} {'arm':<11} {'outcome':<20} {'steps':>13} "
      f"{'path len':>17} {'minGap':>16}")
print("-" * 100)

fails = []
n_checked = 0
for f in sorted(glob.glob("v2/runs/v2/export/trial_*.json")):
    S = Scenario.load(f)
    for arm, (planner, ttc) in ARMS.items():
        key = (S.name, arm)
        if key not in mat:
            continue
        m = mat[key]
        R = run_trial(S, planner, ttc=ttc)
        n_checked += 1

        m_steps = int(float(m["steps"]))
        m_path = float(m["pathLength"])
        m_fd = float(m["finalDist"])
        m_gap = float(m["minPedGap"])

        print(f"{S.name:<18} {arm:<11} {R.outcome + '/' + m['outcome']:<20} "
              f"{R.steps:>6}/{m_steps:<6} {R.path_length:>8.3f}/{m_path:<8.3f} "
              f"{R.min_ped_gap:>7.4f}/{m_gap:<7.4f}")

        checks = [
            ("outcome", R.outcome == m["outcome"]),
            ("steps", abs(R.steps - m_steps) <= TOL["steps_rel"] * max(m_steps, 1)),
            ("path", abs(R.path_length - m_path) <= TOL["path_rel"] * max(m_path, 1e-9)),
            ("final_dist", abs(R.final_dist - m_fd) <= TOL["final_dist"]),
            ("min_gap", abs(R.min_ped_gap - m_gap) <= TOL["min_gap"]),
        ]
        fails += [f"{S.name}/{arm}: {k}" for k, ok in checks if not ok]

print(f"\n{n_checked} arm-scenario pairs checked")
if fails:
    print(f"\n=== {len(fails)} CHECK(S) FAILED ===")
    for x in fails:
        print("  -", x)
    print("\nThe 72-scenario development result must be treated as void until")
    print("this is fixed and rerun from scratch.")
    sys.exit(1)
print("=== ALL CHECKS PASSED — combined pedestrian+obstacle path validated ===")
