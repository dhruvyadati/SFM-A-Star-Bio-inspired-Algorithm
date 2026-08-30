"""The Python planner must reproduce MATLAB's paths EXACTLY.

Run: python3 -m pytest py/test_planner_parity.py -q

Amendment G arm C replans mid-run, so the planner must exist in Python. If that
port differed at all from the MATLAB implementation that produced arms A and
B's initial paths, any B-C difference could be the port rather than replanning
-- the experiment would be confounded at its foundation.

Archived scenarios carry MATLAB-computed waypoints, so parity is checkable
rather than assumed. `atol=0` : grid waypoints are integers and any deviation
is a real difference in the search, not floating-point noise.
"""
import glob
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from planner import GridPlanner, prune_collinear
from sfm_explore import Scenario

def _sample():
    out = []
    for d, per in (("export_fcal", 4), ("export_confirm", 3)):
        for cond in ("baseline", "dense", "cluttered", "bigObs"):
            out += sorted(glob.glob(f"v2/runs/v2/{d}/*{cond}*.json"))[:per]
    return out

CASES = _sample()

@pytest.mark.skipif(not CASES, reason="no scenario files available")
@pytest.mark.parametrize("f", CASES)
def test_waypoints_match_matlab_exactly(f):
    S = Scenario.load(f)
    P = S.params
    gp = GridPlanner(P.xMax, P.yMax, S.obstacles, P.graphMargin)
    path = gp.plan(P.start, P.goal)
    assert path is not None, f"no path found for {f}"
    got = prune_collinear(path)
    ref = S.waypoints
    assert got.shape == ref.shape, (
        f"{Path(f).name}: {got.shape[1]} waypoints vs MATLAB {ref.shape[1]}")
    assert np.array_equal(got, ref), f"{Path(f).name}: waypoint values differ"

@pytest.mark.skipif(not CASES, reason="no scenario files available")
def test_replanning_from_midpath_is_consistent():
    """Replanning from a point ON the original path must yield its tail.

    Arm C replans from the robot's current position. If replanning from an
    on-path node did not reproduce the remaining route, arm C would diverge
    from arm B for reasons unrelated to blockage.
    """
    S = Scenario.load(CASES[0])
    P = S.params
    gp = GridPlanner(P.xMax, P.yMax, S.obstacles, P.graphMargin)
    full = gp.plan(P.start, P.goal)
    mid = full[:, full.shape[1] // 2]
    tail = gp.plan(mid, P.goal)
    assert tail is not None
    # Same cost, not necessarily the same node sequence: ties may resolve
    # differently from a different start. Cost equality is the real invariant.
    cost = lambda p: float(np.sum(np.hypot(np.diff(p[0]), np.diff(p[1]))))
    ref_tail = full[:, full.shape[1] // 2:]
    assert cost(tail) == pytest.approx(cost(ref_tail), abs=1e-9)
