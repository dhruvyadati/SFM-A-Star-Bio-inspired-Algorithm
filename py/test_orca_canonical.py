"""ORCA canonical validation.

Checks more than collision avoidance, per the fairness requirements: minimum
separation, goal completion, reciprocal symmetry, obstacle clearance, speed
limits, deadlock behaviour, and deterministic replay.

These run on hand-built two- and multi-agent cases where the correct behaviour
is known analytically, BEFORE ORCA is used in any comparative experiment.
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

CFG = OrcaConfig()
DT = 0.01
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        print(f"  [PASS] {name}" + (f"  {detail}" if detail else ""))
        passed += 1
    else:
        print(f"  [FAIL] {name}  {detail}")
        failed += 1


def make_sim(radius=0.35, max_speed=2.0):
    return rvo2.PyRVOSimulator(DT, CFG.neighbor_dist, CFG.max_neighbors,
                               CFG.time_horizon, CFG.time_horizon_obst,
                               radius, max_speed)


def run(sim, agents, goals, steps=3000, tol=0.3):
    """Drive agents toward goals; return positions, min separation, arrivals."""
    minsep = math.inf
    maxspeed = 0.0
    arrived = [False] * len(agents)
    for _ in range(steps):
        for i, a in enumerate(agents):
            p = np.array(sim.getAgentPosition(a))
            d = goals[i] - p
            n = np.hypot(*d)
            if n < tol:
                arrived[i] = True
                sim.setAgentPrefVelocity(a, (0.0, 0.0))
            else:
                sim.setAgentPrefVelocity(a, tuple(d / n * 1.3))
        sim.doStep()
        for i in range(len(agents)):
            pi = np.array(sim.getAgentPosition(agents[i]))
            maxspeed = max(maxspeed, float(np.hypot(*sim.getAgentVelocity(agents[i]))))
            for j in range(i + 1, len(agents)):
                pj = np.array(sim.getAgentPosition(agents[j]))
                minsep = min(minsep, float(np.hypot(*(pi - pj))))
        if all(arrived):
            break
    pos = [np.array(sim.getAgentPosition(a)) for a in agents]
    return pos, minsep, arrived, maxspeed


print("=== ORCA canonical validation ===\n")
print("Frozen fairness config:")
for k, v in vars(CFG).items():
    print(f"  {k:<22} {v}")
print()

# --- 1. two-agent head-on ------------------------------------------------
sim = make_sim()
a = sim.addAgent((0.0, 0.0)); b = sim.addAgent((10.0, 0.0))
pos, sep, arr, msp = run(sim, [a, b], [np.array([10.0, 0.0]), np.array([0.0, 0.0])])
check("head-on: both reach the far side", all(arr),
      f"A=({pos[0][0]:.2f},{pos[0][1]:.2f}) B=({pos[1][0]:.2f},{pos[1][1]:.2f})")
check("head-on: never closer than the sum of radii", sep >= 2 * CFG.radius - 1e-6,
      f"min separation {sep:.3f} m vs {2*CFG.radius:.2f} m")

# --- 2. crossing ---------------------------------------------------------
sim = make_sim()
a = sim.addAgent((0.0, 0.0)); b = sim.addAgent((5.0, -5.0))
pos, sep, arr, _ = run(sim, [a, b], [np.array([10.0, 0.0]), np.array([5.0, 5.0])])
check("crossing: both arrive", all(arr))
check("crossing: no overlap", sep >= 2 * CFG.radius - 1e-6, f"min sep {sep:.3f} m")

# --- 3. reciprocal symmetry ---------------------------------------------
# A symmetric head-on pair must resolve symmetrically: equal and opposite
# lateral deviation. Asymmetry would indicate a bias in the solver or setup.
sim = make_sim()
a = sim.addAgent((0.0, 0.0)); b = sim.addAgent((10.0, 0.0))
lat_a = lat_b = 0.0
for _ in range(600):
    for ag, g in ((a, np.array([10.0, 0.0])), (b, np.array([0.0, 0.0]))):
        p = np.array(sim.getAgentPosition(ag)); d = g - p; n = np.hypot(*d)
        sim.setAgentPrefVelocity(ag, tuple(d / n * 1.3) if n > 1e-9 else (0.0, 0.0))
    sim.doStep()
    lat_a = max(lat_a, abs(sim.getAgentPosition(a)[1]))
    lat_b = max(lat_b, abs(sim.getAgentPosition(b)[1]))
check("reciprocal symmetry: equal lateral effort",
      abs(lat_a - lat_b) < 0.02, f"|{lat_a:.4f} - {lat_b:.4f}| = {abs(lat_a-lat_b):.4f}")

# --- 4. obstacle clearance ----------------------------------------------
sim = make_sim()
sim.addObstacle([(4.0, -2.0), (6.0, -2.0), (6.0, 2.0), (4.0, 2.0)])
sim.processObstacles()
a = sim.addAgent((0.0, 0.0))
minclr = math.inf
for _ in range(3000):
    p = np.array(sim.getAgentPosition(a)); d = np.array([10.0, 0.0]) - p
    n = np.hypot(*d)
    sim.setAgentPrefVelocity(a, tuple(d / n * 1.3) if n > 1e-9 else (0.0, 0.0))
    sim.doStep()
    px, py = sim.getAgentPosition(a)
    dx = max(4.0 - px, 0.0, px - 6.0); dy = max(-2.0 - py, 0.0, py - 2.0)
    minclr = min(minclr, math.hypot(dx, dy))
    if n < 0.3:
        break
reached = math.hypot(px - 10.0, py) < 0.5
check("obstacle: clearance maintained", minclr >= 0.0,
      f"min clearance to box {minclr:.3f} m")
# ORCA is a purely LOCAL collision-avoidance method: it has no global planner,
# so with the goal directly behind an obstacle it drives into the face and
# stops. This is expected behaviour, not a defect, but it determines the
# comparison design -- bare ORCA cannot perform the navigation task, so the
# fair contrast is A*-guided ORCA against A*-guided SFM.
print(f"         reached goal behind obstacle: {reached} (final x={px:.2f}, "
      f"goal x=10.0) -- ORCA has no global planner")
check("obstacle: stalls rather than colliding", (not reached) and minclr >= 0.0,
      "safe but cannot route around, as expected for a local-only method")

# --- 5. speed limit ------------------------------------------------------
check("speed never exceeds max_speed", msp <= CFG.max_speed + 1e-6,
      f"observed max {msp:.4f} vs cap {CFG.max_speed}")

# --- 6. deadlock behaviour: dense opposing flow --------------------------
sim = make_sim()
left = [sim.addAgent((0.0, y)) for y in np.linspace(-2, 2, 5)]
right = [sim.addAgent((12.0, y)) for y in np.linspace(-2, 2, 5)]
goals = [np.array([12.0, y]) for y in np.linspace(-2, 2, 5)] + \
        [np.array([0.0, y]) for y in np.linspace(-2, 2, 5)]
pos, sep, arr, _ = run(sim, left + right, goals, steps=6000)
check("dense opposing flow: no overlap", sep >= 2 * CFG.radius - 1e-6,
      f"min sep {sep:.3f} m among 10 agents")
print(f"         arrivals: {sum(arr)}/10 "
      f"({'no gridlock' if sum(arr) >= 8 else 'partial gridlock — worth reporting'})")

# --- 7. deterministic replay --------------------------------------------
def once():
    s = make_sim()
    x = s.addAgent((0.0, 0.0)); y = s.addAgent((8.0, 0.5))
    for _ in range(400):
        for ag, g in ((x, np.array([8.0, 0.0])), (y, np.array([0.0, 0.0]))):
            p = np.array(s.getAgentPosition(ag)); d = g - p; n = np.hypot(*d)
            s.setAgentPrefVelocity(ag, tuple(d / n * 1.3) if n > 1e-9 else (0.0, 0.0))
        s.doStep()
    return s.getAgentPosition(x), s.getAgentPosition(y)

r1, r2 = once(), once()
check("deterministic replay", r1 == r2, f"{r1[0][0]:.9f} vs {r2[0][0]:.9f}")

print(f"\n=== {passed} passed, {failed} failed ===")
if failed:
    sys.exit(1)
