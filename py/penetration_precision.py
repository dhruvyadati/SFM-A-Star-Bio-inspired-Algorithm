"""Full-precision penetration depths for the ORCA-responsive marginal contacts.

The diagnostic printed `%.3f`, which cannot distinguish -0.0005 from -0.0015
and renders anything in that band as "-0.001". Before any claim rests on a 1 mm
figure, the raw values are recomputed and archived at full precision.

Checked in the engine first: `min_ped_gap` is a raw running minimum with no
clipping, and no position correction is applied on penetration. The
`np.maximum(0, .)` inside compute_forces is the contact SPRING FORCE, not the
gap measurement. So there is no simulator-imposed penetration ceiling; whatever
structure appears in the distribution is dynamical.
"""
import json
import sys
from collections import defaultdict
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial
from reciprocal import OrcaPedestrians, PedOrcaConfig

EXPORT = Path("v2/runs/v2/export_confirm")
OUT = Path("v2/runs/v2/penetration_precision.json")


def cat(r):
    if r["outcome"] == "success":
        return "safe" if r["contacts"] == 0 else "unsafe"
    return "deadlock" if r["outcome"] == "deadlock" else "other"


def one(s):
    S = Scenario.load(str(EXPORT / f"{s}.json"))
    drv = OrcaPedestrians(S, PedOrcaConfig(responsibility="reciprocal"))
    R = run_trial(S, "hybrid", log=False, ttc=True, ped_velocity_fn=drv)
    return dict(scenario=s, min_ped_gap=float(R.min_ped_gap),
                min_speed=float(R.min_speed), mean_speed=float(R.mean_speed),
                contacts=int(R.ped_collisions + R.obs_collisions))


if __name__ == "__main__":
    by = defaultdict(dict)
    for r in json.load(open("v2/runs/v2/reciprocal.json")):
        # Key MUST include the set. Dropping it merges the 300 development
        # scenarios into the confirm dicts, and the loader then looks for a
        # dev_* scenario inside export_confirm.
        by[(r["set"], r["responsibility"], r["arm"])][r["scenario"]] = r
    off = by[("confirm", "reciprocal", "disabled")]
    on = by[("confirm", "reciprocal", "ttc0.40")]
    marg = [s for s in sorted(off) if cat(off[s]) == "safe" and cat(on[s]) == "unsafe"]
    assert len(marg) == 191, f"expected 191 marginal scenarios, got {len(marg)}"
    assert all(s.startswith("conf_") for s in marg), "non-confirm scenario selected"
    with Pool(10) as p:
        rows = list(p.imap_unordered(one, marg, chunksize=2))
    OUT.write_text(json.dumps(rows))

    g = np.array([r["min_ped_gap"] for r in rows])
    pen = -g[g < 0]
    print(f"n = {len(g)}   runs with negative gap = {int((g < 0).sum())}")
    print("\nPENETRATION DEPTH (m), full precision")
    for qq in (0, 10, 25, 50, 75, 90, 100):
        print(f"  p{qq:<4} {np.percentile(pen, qq):.9f}")
    print(f"\n  distinct values (9 dp) : {len(np.unique(np.round(pen, 9)))}")
    print(f"  mean {pen.mean():.9f}  sd {pen.std():.9f}")
    print(f"  max/min ratio          : {pen.max() / max(pen.min(), 1e-12):.1f}x")
    print(f"  fraction in [0.5, 1.5] mm : "
          f"{float(np.mean((pen >= 5e-4) & (pen <= 1.5e-3))):.3f}")

    # For undamped unit-mass contact the overlap scales as v / sqrt(kBody),
    # so the depth distribution implies a contact-speed distribution. If these
    # are braked grazes the implied speeds should be very small.
    k = 200.0
    print("\n  implied contact speed  v = overlap * sqrt(kBody):")
    for qq in (10, 50, 90):
        print(f"    p{qq:<3} {np.percentile(pen, qq) * np.sqrt(k):.5f} m/s")
    print(f"  robot nominal speed v0 = 1.30 m/s for scale")
