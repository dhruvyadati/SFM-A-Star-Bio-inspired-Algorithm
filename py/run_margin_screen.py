"""Stage-1 ORCA screen: safety buffer only, all other parameters fixed.

Establishes whether the buffer has a monotonic effect and where the useful
range lies, before spending compute crossing every buffer with every ORCA
parameter. Stage 2 will cross only the plausible margins.

DEVELOPMENT SEEDS ONLY. Confirmation seeds are untouched, and the buffer
selected here is frozen before they are used.

Selection rule, fixed before this ran (see OrcaConfig.select_tolerance_pp):
the SMALLEST buffer whose safe-success rate is within 5 percentage points of
the best configuration, with zero or minimal contact severity. Buffer size is
an explicit cost, so excess conservatism is penalised rather than rewarded.

Covers guided and unguided ORCA, a dense crowd, a cluttered map and large
obstacles, and reports route feasibility and deadlock -- not only pairwise
clearance, which is what the canonical fixtures already established.
"""
import glob
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from orca import OrcaConfig, run_trial_orca
from sfm import Scenario

MARGINS = [0.0, 0.02, 0.05, 0.10, 0.20]
DEV = sorted(glob.glob("v2/runs/v2/export_dev/dev_*.json"))

if not DEV:
    sys.exit("no development scenarios found")

print("=== Stage-1 ORCA margin screen (development only) ===")
print(f"{len(DEV)} scenarios, margins {MARGINS}")
print("mode = ped_margin (buffer confined to agent-agent constraints)\n")

rows = []
for planner in ("hybrid", "direct"):
    for m in MARGINS:
        cfg = OrcaConfig(responsibility="ped_margin", responsibility_margin=m)
        out = {"success": 0, "deadlock": 0, "timeout": 0}
        contacts = 0
        worst_pen = 0.0
        safe = 0
        times = []
        for f in DEV:
            S = Scenario.load(f)
            R = run_trial_orca(S, mode="orca_robot", cfg=cfg, planner=planner)
            out[R.outcome] = out.get(R.outcome, 0) + 1
            ncol = R.ped_collisions + R.obs_collisions
            contacts += ncol
            # Penetration measured with TRUE radii, never the inflated ones.
            worst_pen = max(worst_pen, max(0.0, -R.min_ped_gap))
            if R.outcome == "success" and ncol == 0:
                safe += 1
                times.append(R.steps * S.params.dt)
        n = len(DEV)
        rows.append(dict(planner=planner, margin=m,
                         safe=100.0 * safe / n,
                         goal=100.0 * out["success"] / n,
                         dead=100.0 * out["deadlock"] / n,
                         contacts=contacts / n,
                         pen=worst_pen,
                         t=float(np.mean(times)) if times else float("nan")))
        print(f"  planner={planner:<7} margin={m:.2f}  "
              f"safe={rows[-1]['safe']:5.1f}%  goal={rows[-1]['goal']:5.1f}%  "
              f"dead={rows[-1]['dead']:5.1f}%  contacts/trial={rows[-1]['contacts']:.2f}  "
              f"maxPen={worst_pen:.4f} m")

print()
print(f"{'planner':<9} {'margin':>7} {'safe%':>7} {'goal%':>7} {'dead%':>7} "
      f"{'cont/trial':>11} {'maxPen m':>9} {'mean t s':>9}")
print("-" * 72)
for r in rows:
    print(f"{r['planner']:<9} {r['margin']:>7.2f} {r['safe']:>7.1f} {r['goal']:>7.1f} "
          f"{r['dead']:>7.1f} {r['contacts']:>11.2f} {r['pen']:>9.4f} {r['t']:>9.1f}")

# --- selection, guided arm ------------------------------------------------
tol = OrcaConfig().select_tolerance_pp
guided = [r for r in rows if r["planner"] == "hybrid"]
best = max(r["safe"] for r in guided)
eligible = [r for r in guided if r["safe"] >= best - tol]
eligible.sort(key=lambda r: (r["contacts"] > 0, r["margin"]))
pick = eligible[0]

print(f"\nBest guided safe-success: {best:.1f}%  (tolerance {tol:.0f} pp)")
print(f"Eligible margins: {[r['margin'] for r in eligible]}")
print(f"SELECTED buffer: {pick['margin']:.2f} m  "
      f"(safe {pick['safe']:.1f}%, contacts/trial {pick['contacts']:.2f})")

mono = all(guided[i]["contacts"] >= guided[i + 1]["contacts"] - 1e-9
           for i in range(len(guided) - 1))
print(f"\nContact rate monotonically non-increasing in buffer: {mono}")
print("Stage 2 should cross only the plausible margins with timeHorizon,")
print("timeHorizonObst, neighborDist and maxNeighbors.")
