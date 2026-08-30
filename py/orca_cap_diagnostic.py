"""Does the ORCA ten-neighbour cap bind, and does it bind unevenly by condition?

`max_neighbors = 10` means ORCA considers at most ten agent neighbours per
pedestrian per step. In the dense condition a pedestrian may have more than ten
others within the 5 m neighbour distance, in which case only a subset
constrains its velocity and pedestrians may respond less strongly than the
parameter nominally suggests.

This does NOT invalidate the study. The cap is part of the definition of the
ORCA-responsive pedestrian model that was actually tested. It does bound how
far dense-condition conclusions generalise, so it is measured and disclosed
rather than assumed negligible.

The frozen study is untouched. This replays a sample with an instrumented
wrapper; the driver is deterministic, so the replay reproduces the same
trajectories the study produced.

If the cap binds frequently, a later sensitivity check at a larger value would
be informative. It would be reported as MODEL SENSITIVITY and must not be
substituted into the frozen study.
"""
from __future__ import annotations

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
OUT = Path("v2/runs/v2/orca_cap_diagnostic.json")
PER_CONDITION = 15
WORKERS = 10


class Counting(OrcaPedestrians):
    """Wraps the driver and counts neighbours within the ORCA neighbour radius.

    Counts other AGENTS (pedestrians plus the robot), which is what ORCA's
    max_neighbors limits. Obstacles have a separate limit and are not counted.
    """

    def __init__(self, S, cfg):
        super().__init__(S, cfg)
        self.samples = 0          # (pedestrian, step) pairs
        self.over = 0             # pairs where the cap bound
        self.max_seen = 0
        self.hist = defaultdict(int)

    def __call__(self, step, x, v, radii, goals):
        # Every step is sampled; the count is over agents within neighbour
        # distance, excluding self.
        d = np.hypot(x[0][:, None] - x[0][None, :], x[1][:, None] - x[1][None, :])
        near = (d < self.cfg.neighbor_dist)
        np.fill_diagonal(near, False)
        counts = near[1:].sum(axis=1)          # pedestrians only, robot included as a column
        self.samples += len(counts)
        self.over += int(np.sum(counts > self.cfg.max_neighbors))
        self.max_seen = max(self.max_seen, int(counts.max()) if len(counts) else 0)
        for c in counts:
            self.hist[int(c)] += 1
        return super().__call__(step, x, v, radii, goals)


def one(job):
    scen, cond = job
    S = Scenario.load(str(EXPORT / f"{scen}.json"))
    drv = Counting(S, PedOrcaConfig())
    run_trial(S, "hybrid", log=False, ttc=True, ped_velocity_fn=drv)
    return dict(scenario=scen, condition=cond, n_ped=int(S.ped_pos.shape[1]),
                samples=drv.samples, over=drv.over, max_seen=drv.max_seen,
                frac=drv.over / max(drv.samples, 1))


if __name__ == "__main__":
    scens = sorted(p.stem for p in EXPORT.glob("conf_*.json"))
    bycond = defaultdict(list)
    for s in scens:
        bycond[s.split("_")[1]].append(s)
    jobs = [(s, c) for c, ss in sorted(bycond.items()) for s in ss[:PER_CONDITION]]
    print(f"replaying {len(jobs)} scenarios ({PER_CONDITION} per condition)", flush=True)

    with Pool(WORKERS) as pool:
        rows = list(pool.imap_unordered(one, jobs, chunksize=1))
    OUT.write_text(json.dumps(rows))

    print("\n" + "=" * 78)
    print("ORCA ten-neighbour cap: how often does it bind?")
    print("=" * 78)
    print(f"  {'condition':<12}{'scen':>6}{'peds':>7}{'cap-bound %':>13}{'max neighbours':>16}")
    print("  " + "-" * 60)
    allfrac = []
    for c in sorted({r["condition"] for r in rows}):
        rr = [r for r in rows if r["condition"] == c]
        f = 100 * sum(r["over"] for r in rr) / max(sum(r["samples"] for r in rr), 1)
        allfrac.append((c, f))
        print(f"  {c:<12}{len(rr):>6}{np.mean([r['n_ped'] for r in rr]):>7.0f}"
              f"{f:>13.2f}{max(r['max_seen'] for r in rr):>16}")
    tot = 100 * sum(r["over"] for r in rows) / max(sum(r["samples"] for r in rows), 1)
    print(f"\n  overall cap-bound: {tot:.2f}% of (pedestrian, step) samples")
    spread = max(f for _, f in allfrac) - min(f for _, f in allfrac)
    print(f"  spread across conditions: {spread:.2f} percentage points")
    print("\n  Reporting rule: if the cap binds often, or much more in one")
    print("  condition, dense-condition conclusions are bounded by the cap and")
    print("  a larger-value sensitivity check is warranted — as MODEL")
    print("  SENSITIVITY, never substituted into the frozen study.")
