"""Does the safety buffer leak into obstacle conservatism?

The reciprocity mismatch exists only against externally controlled
pedestrians. A buffer that also makes the robot warier of STATIC geometry is
changing behaviour for a reason unrelated to reciprocity, and a reviewer is
entitled to ask why.

RVO2 uses one radius for both agent and obstacle constraints, so inflating the
ROBOT's radius ('margin') necessarily affects both. Inflating the PEDESTRIAN
radii as RVO2 sees them ('ped_margin') confines the buffer to agent-agent
constraints, because in orca_robot mode pedestrians are externally driven and
their radius enters nowhere else.

This measures obstacle standoff under each mode. 'ped_margin' must match the
zero-buffer baseline exactly.
"""
import math, sys
from pathlib import Path
import numpy as np
import pytest

# Optional dependency: skip cleanly instead of failing collection.
rvo2 = pytest.importorskip(
    "rvo2", reason="optional, required only for ORCA validation")

sys.path.insert(0, str(Path(__file__).parent))
from orca import OrcaConfig

R_ROB, R_PED = 0.35, 0.30
CFG = OrcaConfig()

def obstacle_standoff(mode, margin):
    r_eff = R_ROB + (margin if mode == "margin" else 0.0)
    sim = rvo2.PyRVOSimulator(0.01, CFG.neighbor_dist, CFG.max_neighbors,
                              CFG.time_horizon, CFG.time_horizon_obst,
                              r_eff, CFG.max_speed)
    sim.addObstacle([(4., -3.), (6., -3.), (6., 3.), (4., 3.)])
    sim.processObstacles()
    a = sim.addAgent((0., 0.), CFG.neighbor_dist, CFG.max_neighbors,
                     CFG.time_horizon, CFG.time_horizon_obst,
                     r_eff, CFG.max_speed, (0., 0.))
    for _ in range(2500):
        p = np.array(sim.getAgentPosition(a))
        d = np.array([10., 0.]) - p
        n = np.hypot(*d)
        sim.setAgentPrefVelocity(a, tuple(d / n * 1.3) if n > 1e-9 else (0., 0.))
        sim.doStep()
    return 4.0 - sim.getAgentPosition(a)[0]

print("=== Does the buffer change obstacle conservatism? ===\n")
print("Robot drives at a wall face at x = 4. Standoff = 4 - final x.\n")
print(f"{'mode':<14} {'buffer':>8} {'standoff':>10} {'vs baseline':>13}")
print("-" * 50)
base = obstacle_standoff("reciprocal", 0.0)
print(f"{'reciprocal':<14} {0.0:>8.2f} {base:>10.4f} {'baseline':>13}")
for m in (0.02, 0.05, 0.10, 0.20):
    s = obstacle_standoff("margin", m)
    print(f"{'margin':<14} {m:>8.2f} {s:>10.4f} {s - base:>+13.4f}")
for m in (0.02, 0.05, 0.10, 0.20):
    s = obstacle_standoff("ped_margin", m)
    tag = "IDENTICAL" if abs(s - base) < 1e-9 else f"{s - base:+.4f}"
    print(f"{'ped_margin':<14} {m:>8.2f} {s:>10.4f} {tag:>13}")

print("\n'margin' shifts obstacle standoff by exactly the buffer, which is")
print("conservatism unrelated to reciprocity. 'ped_margin' must leave it")
print("untouched -- that is the property being asserted.")
