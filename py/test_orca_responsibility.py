"""Which responsibility mode should ORCA use against non-yielding pedestrians?

ORCA's half-plane construction assumes both parties contribute avoidance
effort. SFM pedestrians do not run ORCA and will not, so the assumption fails
and the robot must make up the difference. There is no exact way to express
that through the RVO2 public API, so the three candidates are compared here on
cases where the correct behaviour is unambiguous, BEFORE any comparative
experiment and using development geometry only.

The decisive case is a pedestrian marching straight at the robot on a fixed
course. A reciprocal-assuming robot will plan for half the required evasion,
the pedestrian will supply none, and separation will be lost.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Optional dependency: skip cleanly instead of failing collection.
rvo2 = pytest.importorskip(
    "rvo2", reason="optional, required only for ORCA validation")


sys.path.insert(0, str(Path(__file__).parent))
from orca import OrcaConfig

DT = 0.01
R_ROB, R_PED = 0.35, 0.30
CONTACT = R_ROB + R_PED

MODES = ["reciprocal", "frozen", "margin"]


def run_case(mode, ped_start, ped_vel, robot_start, goal, steps=1500,
             margin=0.20):
    cfg = OrcaConfig()
    r_eff = R_ROB + (margin if mode == "margin" else 0.0)
    sim = rvo2.PyRVOSimulator(DT, cfg.neighbor_dist, cfg.max_neighbors,
                              cfg.time_horizon, cfg.time_horizon_obst,
                              r_eff, cfg.max_speed)
    rid = sim.addAgent(tuple(robot_start), cfg.neighbor_dist,
                       cfg.max_neighbors, cfg.time_horizon,
                       cfg.time_horizon_obst, r_eff, cfg.max_speed, (0.0, 0.0))
    pid = sim.addAgent(tuple(ped_start), cfg.neighbor_dist, cfg.max_neighbors,
                       cfg.time_horizon, cfg.time_horizon_obst,
                       R_PED, 1.34, tuple(ped_vel))
    if mode == "frozen":
        sim.setAgentMaxSpeed(pid, 0.0)

    p_ped = np.asarray(ped_start, dtype=float)
    minsep = math.inf
    reached = False
    for _ in range(steps):
        # The pedestrian is EXTERNALLY driven on a fixed course and never
        # yields, which is the whole point of the test.
        p_ped = p_ped + np.asarray(ped_vel) * DT
        sim.setAgentPosition(pid, (float(p_ped[0]), float(p_ped[1])))
        sim.setAgentVelocity(pid, tuple(ped_vel))

        rp = np.array(sim.getAgentPosition(rid))
        d = np.asarray(goal, dtype=float) - rp
        n = np.hypot(*d)
        sim.setAgentPrefVelocity(rid, tuple(d / n * 1.3) if n > 1e-9 else (0., 0.))
        sim.doStep()

        rp = np.array(sim.getAgentPosition(rid))
        minsep = min(minsep, float(np.hypot(*(rp - p_ped))))
        if np.hypot(*(np.asarray(goal) - rp)) < 0.5:
            reached = True
            break
    return minsep, reached, np.array(sim.getAgentPosition(rid))


print("=== ORCA responsibility mode selection (development only) ===\n")
print(f"contact distance = {CONTACT:.2f} m\n")

cases = {
    "head-on, non-yielding":  (np.array([12., 0.]), np.array([-1.34, 0.]),
                               np.array([0., 0.]), np.array([12., 0.])),
    "crossing, non-yielding": (np.array([6., -6.]), np.array([0., 1.34]),
                               np.array([0., 0.]), np.array([12., 0.])),
    "overtaking, slow ped":   (np.array([4., 0.05]), np.array([0.4, 0.]),
                               np.array([0., 0.]), np.array([14., 0.])),
}

results = {m: {} for m in MODES}
for cname, (ps, pv, rs, g) in cases.items():
    print(f"{cname}")
    for m in MODES:
        sep, reached, fin = run_case(m, ps, pv, rs, g)
        results[m][cname] = (sep, reached)
        status = "CONTACT" if sep < CONTACT else "clear"
        print(f"   {m:<12} min sep {sep:6.3f} m  {status:<8} "
              f"reached goal: {reached}")
    print()

# --- selection --------------------------------------------------------
# Lexicographic, safety first: never contact, then reach the goal, then the
# least excess clearance -- a mode that stays far away by refusing to move is
# not preferable.
print("Selection, lexicographic: no contact -> reaches goal -> least excess "
      "clearance\n")
print(f"{'mode':<12} {'contacts':>9} {'goals reached':>14} {'mean sep':>10}")
print("-" * 50)
best, best_key = None, None
for m in MODES:
    contacts = sum(1 for s, _ in results[m].values() if s < CONTACT)
    goals = sum(1 for _, r in results[m].values() if r)
    msep = float(np.mean([s for s, _ in results[m].values()]))
    print(f"{m:<12} {contacts:>9} {goals:>14} {msep:>10.3f}")
    key = (contacts, -goals, msep)
    if best is None or key < best_key:
        best, best_key = m, key

print(f"\nSelected: {best}")
print("\nNone of these is exact ORCA-with-full-responsibility -- RVO2 exposes no")
print("responsibility parameter. Any write-up must state that the robot arm")
print("uses an approximate non-reciprocal treatment, name which one, and report")
print("this comparison as its justification.")
