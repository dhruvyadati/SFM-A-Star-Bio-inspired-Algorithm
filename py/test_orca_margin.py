"""Two questions about the robot-side safety margin, before it is tuned.

1. ARE THE OBSERVED OVERLAPS REAL? The reciprocal mode lost separation by
   9 mm and 1 mm in the selection fixtures. At dt = 0.01 s and closing speeds
   near 2.6 m/s the robot moves ~2.6 cm per step, so a millimetre-scale
   overlap could be a discrete-sampling artifact rather than a real contact.
   Checked with swept separation and a timestep sweep.

2. DOES THE MARGIN BLOCK CORRIDORS? A 0.20 m margin adds 0.40 m to the
   effective diameter. That may make feasible passages appear blocked and
   convert collisions into deadlocks, which would be conservatism bought at
   the cost of progress rather than a safety gain.
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

R_ROB, R_PED = 0.35, 0.30
CONTACT = R_ROB + R_PED
CFG = OrcaConfig()


def seg_min_dist(a0, a1, b0, b1):
    """Minimum distance between two points over one step, both moving."""
    r0 = np.asarray(b0) - np.asarray(a0)
    r1 = np.asarray(b1) - np.asarray(a1)
    w = r1 - r0
    ww = float(w @ w)
    s = 0.0 if ww < 1e-18 else min(1.0, max(0.0, float(-(r0 @ w) / ww)))
    return float(np.hypot(*(r0 + s * w)))


def head_to_head(mode, dt, margin=0.20, steps_scale=1.0):
    r_eff = R_ROB + (margin if mode == "margin" else 0.0)
    sim = rvo2.PyRVOSimulator(dt, CFG.neighbor_dist, CFG.max_neighbors,
                              CFG.time_horizon, CFG.time_horizon_obst,
                              r_eff, CFG.max_speed)
    rid = sim.addAgent((0., 0.), CFG.neighbor_dist, CFG.max_neighbors,
                       CFG.time_horizon, CFG.time_horizon_obst,
                       r_eff, CFG.max_speed, (0., 0.))
    pid = sim.addAgent((6., -6.), CFG.neighbor_dist, CFG.max_neighbors,
                       CFG.time_horizon, CFG.time_horizon_obst,
                       R_PED, 1.34, (0., 1.34))
    p_ped = np.array([6., -6.])
    pv = np.array([0., 1.34])
    sampled = swept = math.inf
    prev_r = np.array(sim.getAgentPosition(rid))
    prev_p = p_ped.copy()
    for _ in range(int(1500 * steps_scale)):
        p_ped = p_ped + pv * dt
        sim.setAgentPosition(pid, tuple(p_ped))
        sim.setAgentVelocity(pid, tuple(pv))
        rp = np.array(sim.getAgentPosition(rid))
        d = np.array([12., 0.]) - rp
        n = np.hypot(*d)
        sim.setAgentPrefVelocity(rid, tuple(d / n * 1.3) if n > 1e-9 else (0., 0.))
        sim.doStep()
        rp = np.array(sim.getAgentPosition(rid))
        sampled = min(sampled, float(np.hypot(*(rp - p_ped))))
        swept = min(swept, seg_min_dist(prev_r, rp, prev_p, p_ped))
        prev_r, prev_p = rp, p_ped.copy()
        if np.hypot(*(np.array([12., 0.]) - rp)) < 0.5:
            break
    return sampled, swept


print("=== 1. Are the millimetre overlaps real? ===\n")
print(f"contact distance {CONTACT:.3f} m\n")
print(f"{'mode':<12} {'dt':>8} {'sampled':>10} {'swept':>10} {'verdict':>10}")
print("-" * 54)
for mode in ("reciprocal", "margin"):
    for dt in (0.01, 0.005, 0.0025):
        s, w = head_to_head(mode, dt, steps_scale=0.01 / dt)
        verdict = "CONTACT" if w < CONTACT else "clear"
        print(f"{mode:<12} {dt:>8.4f} {s:>10.4f} {w:>10.4f} {verdict:>10}")
print("\nIf swept < sampled the discrete check was optimistic. If the verdict")
print("flips with dt, the overlap was a discretisation artifact rather than a")
print("real loss of separation.\n")

print("=== 2. Does the margin block corridors? ===\n")
print(f"{'corridor width':>15} {'margin 0.00':>14} {'margin 0.10':>14} {'margin 0.20':>14}")
print("-" * 60)
for width in (1.2, 1.6, 2.0, 2.5, 3.0):
    row = f"{width:>15.1f}"
    for margin in (0.0, 0.10, 0.20):
        r_eff = R_ROB + margin
        sim = rvo2.PyRVOSimulator(0.01, CFG.neighbor_dist, CFG.max_neighbors,
                                  CFG.time_horizon, CFG.time_horizon_obst,
                                  r_eff, CFG.max_speed)
        half = width / 2.0
        # Two walls forming a corridor along y = 0 from x = 4 to x = 8.
        sim.addObstacle([(4., half), (8., half), (8., half + 3), (4., half + 3)])
        sim.addObstacle([(4., -half - 3), (8., -half - 3), (8., -half), (4., -half)])
        sim.processObstacles()
        a = sim.addAgent((0., 0.), CFG.neighbor_dist, CFG.max_neighbors,
                         CFG.time_horizon, CFG.time_horizon_obst,
                         r_eff, CFG.max_speed, (0., 0.))
        passed_thru = False
        for _ in range(4000):
            p = np.array(sim.getAgentPosition(a))
            d = np.array([12., 0.]) - p
            n = np.hypot(*d)
            sim.setAgentPrefVelocity(a, tuple(d / n * 1.3) if n > 1e-9 else (0., 0.))
            sim.doStep()
            if sim.getAgentPosition(a)[0] > 9.0:
                passed_thru = True
                break
        row += f"{('pass' if passed_thru else 'BLOCKED'):>14}"
    print(row)

print("\nGeometric feasibility: a corridor of width W admits a disc of radius r")
print("when W > 2r. margin 0.20 gives r_eff = 0.55, so W must exceed 1.10 m")
print("for control, while the true body needs only 0.70 m.")
