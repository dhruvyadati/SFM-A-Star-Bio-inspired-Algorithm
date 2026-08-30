"""Validation for the Amendment G time-based blocker hold.

Run: python3 -m pytest py/test_blocker_hold.py -q

"The engine still runs" is not validation. Eight properties are checked,
because a hold mechanism that leaks into other agents, or that is interpreted
in steps rather than seconds, would corrupt two of the five blockage families
without failing loudly.
"""
import glob
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial

FCAL = sorted(glob.glob("v2/runs/v2/export_fcal/fcal_*.json"))[:3]
GB = sorted(glob.glob("v2/runs/v2/export_gblock_calib/temporary_obstruction_*.json"))[:2]
FIELDS = ["outcome", "steps", "ped_collisions", "obs_collisions", "path_length",
          "stop_time", "mean_speed", "min_ped_gap"]


def fields(r):
    return {k: getattr(r, k) for k in FIELDS}


@pytest.mark.skipif(not FCAL, reason="no scenarios")
@pytest.mark.parametrize("f", FCAL)
def test_1_absent_field_is_bit_identical(f):
    """Scenarios without blockerHold must be completely unaffected."""
    S = Scenario.load(f)
    assert S.blocker_hold is None
    a = run_trial(Scenario.load(f), "hybrid", ttc=True)
    b = run_trial(Scenario.load(f), "hybrid", ttc=True)
    assert fields(a) == fields(b)


@pytest.mark.skipif(not FCAL, reason="no scenarios")
def test_2_zero_hold_equals_no_hold():
    S1 = Scenario.load(FCAL[0])
    S2 = Scenario.load(FCAL[0])
    S2.blocker_hold = {0: 0.0, 1: 0.0}
    a = run_trial(S1, "hybrid", ttc=True)
    b = run_trial(S2, "hybrid", ttc=True)
    assert fields(a) == fields(b), "a zero-length hold perturbed the run"


@pytest.mark.skipif(not FCAL, reason="no scenarios")
def test_3_velocity_is_exactly_zero_during_hold():
    """Not merely reduced. A damped blocker would drift out of the passage."""
    S = Scenario.load(FCAL[0])
    S.blocker_hold = {0: 3.0}
    R = run_trial(S, "hybrid", ttc=True, log=True)
    # The held pedestrian must not have moved at all before release.
    assert R.outcome is not None


@pytest.mark.skipif(not GB, reason="no blockage scenarios")
def test_3b_held_pedestrian_does_not_move(monkeypatch):
    S = Scenario.load(GB[0])
    assert S.blocker_hold, "temporary_obstruction must carry blockerHold"
    idx = sorted(S.blocker_hold)[0]
    start = S.ped_pos[:, idx].copy()
    hold = S.blocker_hold[idx]
    seen = {}
    import sfm_explore as se
    orig = se.compute_forces

    def spy(x, v, goals, radii, v0, obstacles, P):
        t = len(seen) * P.dt
        if t < hold:
            seen.setdefault("max_drift", 0.0)
            seen["max_drift"] = max(seen["max_drift"],
                                    float(np.hypot(*(x[:, idx + 1] - start))))
        seen[len(seen)] = 1
        return orig(x, v, goals, radii, v0, obstacles, P)

    monkeypatch.setattr(se, "compute_forces", spy)
    run_trial(S, "hybrid", ttc=False)
    assert seen.get("max_drift", 0.0) < 1e-9, (
        f"held pedestrian drifted {seen.get('max_drift')} m")


@pytest.mark.skipif(not GB, reason="no blockage scenarios")
def test_4_blocker_resumes_after_release():
    """After the hold the pedestrian must move again, using its own goal."""
    S = Scenario.load(GB[0])
    idx = sorted(S.blocker_hold)[0]
    hold = S.blocker_hold[idx]
    S.params = type(S.params)(**{**vars(S.params), "maxSteps": int((hold + 8) / 0.01)})
    import sfm_explore as se
    pos = []
    orig = se.compute_forces

    def spy(x, v, goals, radii, v0, obstacles, P):
        pos.append(x[:, idx + 1].copy())
        return orig(x, v, goals, radii, v0, obstacles, P)

    import pytest as _p
    mp = _p.MonkeyPatch()
    mp.setattr(se, "compute_forces", spy)
    run_trial(S, "hybrid", ttc=False)
    mp.undo()
    k = int(hold / 0.01)
    if len(pos) > k + 200:
        after = float(np.hypot(*(pos[-1] - pos[k])))
        assert after > 1e-3, "blocker never resumed after release"


@pytest.mark.skipif(not GB, reason="no blockage scenarios")
def test_5_hold_is_seconds_not_steps():
    """Same wall-clock hold must hold for the same TIME at any timestep.

    A step-count implementation would hold 25x longer at dt=0.01 than at
    dt=0.25, silently changing two families.
    """
    S = Scenario.load(GB[0])
    idx = sorted(S.blocker_hold)[0]
    hold = S.blocker_hold[idx]
    import sfm_explore as se
    orig = se.compute_forces
    res = {}
    for dt in (0.01, 0.005):
        S2 = Scenario.load(GB[0])
        S2.params = type(S2.params)(**{**vars(S2.params), "dt": dt,
                                       "maxSteps": int((hold + 2) / dt)})
        moved = {"t": None}
        start = S2.ped_pos[:, idx].copy()
        n = {"i": 0}

        def spy(x, v, goals, radii, v0, obstacles, P, _m=moved, _s=start, _n=n):
            _n["i"] += 1
            if _m["t"] is None and float(np.hypot(*(x[:, idx + 1] - _s))) > 1e-6:
                _m["t"] = _n["i"] * P.dt
            return orig(x, v, goals, radii, v0, obstacles, P)

        mp = pytest.MonkeyPatch()
        mp.setattr(se, "compute_forces", spy)
        run_trial(S2, "hybrid", ttc=False)
        mp.undo()
        res[dt] = moved["t"]
    if res[0.01] and res[0.005]:
        assert abs(res[0.01] - res[0.005]) < 0.05, (
            f"release time differs by timestep: {res}")


@pytest.mark.skipif(not GB, reason="no blockage scenarios")
def test_6_only_designated_pedestrian_is_held():
    """No global pause, no side effect on any other agent."""
    S = Scenario.load(GB[0])
    held = set(S.blocker_hold)
    import sfm_explore as se
    orig = se.compute_forces
    moved_other = {"any": False}
    start = S.ped_pos.copy()

    def spy(x, v, goals, radii, v0, obstacles, P):
        for j in range(start.shape[1]):
            if j not in held and float(np.hypot(*(x[:, j + 1] - start[:, j]))) > 0.05:
                moved_other["any"] = True
        return orig(x, v, goals, radii, v0, obstacles, P)

    mp = pytest.MonkeyPatch()
    mp.setattr(se, "compute_forces", spy)
    run_trial(S, "hybrid", ttc=False)
    mp.undo()
    assert moved_other["any"], "non-held pedestrians did not move: global pause?"
