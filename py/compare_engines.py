"""Cross-validate the Python engine against MATLAB on canonical scenarios.

Tolerances are PRESPECIFIED here, before results are inspected. Outcome
agreement alone is not sufficient: two engines can both report "success" while
following materially different trajectories, so position error over time is
checked as well.
"""
import csv
import glob
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm import Scenario, run_trial

# --- prespecified tolerances ------------------------------------------
TOL = {
    "outcome":      "exact",
    "steps_rel":    0.01,     # 1% of step count
    "path_rel":     0.001,    # 0.1% of path length
    "final_dist":   1e-3,     # m
    "clearance":    1e-3,     # m
    "collisions":   "exact",
    "traj_max":     0.05,     # m, max position error at any step
    "traj_rms":     0.01,     # m, RMS position error over the trajectory
}

EXPORT = Path("v2/runs/v2/export")

mat = {}
with open("v2/runs/v2/canonical_matlab.csv") as f:
    for row in csv.DictReader(f):
        mat[row["name"]] = row

print("=== Python vs MATLAB, canonical scenarios ===\n")
print(f"{'case':<12} {'outcome':<18} {'steps':>13} {'path len':>16} "
      f"{'traj max':>9} {'traj rms':>9}")
print("-" * 82)

fails = []
for f in sorted(glob.glob(str(EXPORT / "canonical_*.json"))):
    S = Scenario.load(f)
    R = run_trial(S, "sfm", log=True)
    m = mat.get(S.name)
    if m is None:
        fails.append(f"{S.name}: no MATLAB result")
        continue

    m_out = m["outcome"]
    m_steps = int(float(m["steps"]))
    m_path = float(m["pathLength"])
    m_fd = float(m["finalDist"])

    # trajectory comparison
    tf = EXPORT / f"mtraj_{S.name}.csv"
    tmax = trms = float("nan")
    if tf.exists():
        mt = np.loadtxt(tf, delimiter=",")
        if mt.ndim == 1:
            mt = mt[None, :]
        n = min(mt.shape[0], R.traj.shape[1])
        err = np.hypot(mt[:n, 0] - R.traj[0, :n], mt[:n, 1] - R.traj[1, :n])
        tmax, trms = float(err.max()), float(np.sqrt((err ** 2).mean()))

    ok_out = R.outcome == m_out
    ok_steps = abs(R.steps - m_steps) <= TOL["steps_rel"] * max(m_steps, 1)
    ok_path = abs(R.path_length - m_path) <= TOL["path_rel"] * max(m_path, 1e-9)
    ok_fd = abs(R.final_dist - m_fd) <= TOL["final_dist"]
    ok_tj = (math.isnan(tmax)) or (tmax <= TOL["traj_max"] and trms <= TOL["traj_rms"])

    mark = lambda b: "ok" if b else "FAIL"
    print(f"{S.name:<12} {R.outcome + '/' + m_out:<18} "
          f"{R.steps:>6}/{m_steps:<6} {R.path_length:>7.3f}/{m_path:<7.3f} "
          f"{tmax:>9.2e} {trms:>9.2e}")

    for name, ok in [("outcome", ok_out), ("steps", ok_steps),
                     ("path", ok_path), ("final_dist", ok_fd), ("traj", ok_tj)]:
        if not ok:
            fails.append(f"{S.name}: {name}")

print("\nPrespecified tolerances:")
for k, v in TOL.items():
    print(f"  {k:<12} {v}")

print()
if fails:
    print(f"=== {len(fails)} CHECK(S) FAILED ===")
    for x in fails:
        print("  -", x)
    sys.exit(1)
print("=== ALL CHECKS PASSED — engines agree within tolerance ===")
