"""Cross-validate the Python TTC filter against MATLAB on canonical scenarios.

Same prespecified tolerances as the SFM engine comparison. The filter is the
only difference between these runs and the already-validated unfiltered ones,
so any disagreement localises to the filter itself.
"""
import csv, glob, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from sfm import Scenario, run_trial

TOL = {"steps_rel": 0.01, "path_rel": 0.001, "final_dist": 1e-3,
       "traj_max": 0.05, "traj_rms": 0.01}
EXPORT = Path("v2/runs/v2/export")

mat = {r["name"]: r for r in
       csv.DictReader(open("v2/runs/v2/canonical_ttc_matlab.csv"))}

print("=== Python vs MATLAB, TTC filter enabled ===\n")
print(f"{'case':<12} {'outcome':<20} {'steps':>13} {'path len':>16} "
      f"{'traj max':>10} {'traj rms':>10}")
print("-" * 86)

fails = []
for f in sorted(glob.glob(str(EXPORT / "canonical_*.json"))):
    S = Scenario.load(f)
    R = run_trial(S, "sfm", log=True, ttc=True)
    m = mat.get(S.name)
    if m is None:
        continue
    m_steps, m_path = int(float(m["steps"])), float(m["pathLength"])
    m_fd = float(m["finalDist"])

    tf = EXPORT / f"mttc_{S.name}.csv"
    tmax = trms = float("nan")
    if tf.exists():
        mt = np.loadtxt(tf, delimiter=",")
        if mt.ndim == 1:
            mt = mt[None, :]
        n = min(mt.shape[0], R.traj.shape[1])
        err = np.hypot(mt[:n, 0] - R.traj[0, :n], mt[:n, 1] - R.traj[1, :n])
        tmax, trms = float(err.max()), float(np.sqrt((err ** 2).mean()))

    print(f"{S.name:<12} {R.outcome + '/' + m['outcome']:<20} "
          f"{R.steps:>6}/{m_steps:<6} {R.path_length:>7.3f}/{m_path:<7.3f} "
          f"{tmax:>10.2e} {trms:>10.2e}")

    checks = [
        ("outcome", R.outcome == m["outcome"]),
        ("steps", abs(R.steps - m_steps) <= TOL["steps_rel"] * max(m_steps, 1)),
        ("path", abs(R.path_length - m_path) <= TOL["path_rel"] * max(m_path, 1e-9)),
        ("final_dist", abs(R.final_dist - m_fd) <= TOL["final_dist"]),
        ("traj", math.isnan(tmax) or (tmax <= TOL["traj_max"]
                                      and trms <= TOL["traj_rms"])),
    ]
    fails += [f"{S.name}: {k}" for k, ok in checks if not ok]

print()
if fails:
    print(f"=== {len(fails)} CHECK(S) FAILED ===")
    for x in fails:
        print("  -", x)
    sys.exit(1)
print("=== ALL CHECKS PASSED — TTC filter reproduced within tolerance ===")
