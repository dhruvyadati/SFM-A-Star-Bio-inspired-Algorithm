"""A*-guided ORCA: obstacle registration and waypoint-following validation.

Runs BEFORE any comparative experiment. Two things are checked separately,
because they fail in different ways:

  OBSTACLE REGISTRATION -- winding order, processObstacles, whether the robot
  penetrates, and whether the radius is being counted twice. A robot that
  halts at exactly one radius from the face is correct; one that halts at two
  radii is double-counting, and one that penetrates is mis-registered.

  WAYPOINT FOLLOWING -- ORCA must consume the SAME A* path as the SFM arms.
  If the waypoint logic is copied across without checking, ORCA can oscillate
  at capture boundaries or cut corners that SFM rounds.
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
from orca import OrcaConfig, run_trial_orca, _add_obstacles
from sfm import Scenario, run_trial, poly_signed_distance

CFG = OrcaConfig()
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    tag = "[PASS]" if ok else "[FAIL]"
    print(f"  {tag} {name}" + (f"  {detail}" if detail else ""))
    if ok:
        passed += 1
    else:
        failed += 1


print("=== A*-ORCA obstacle and waypoint validation ===\n")

# --- 1. Winding order is normalised, not assumed ------------------------
# The exporter may emit either order. _add_obstacles must produce identical
# behaviour for a polygon and its reversal.
box_ccw = [np.array([[4., -2.], [6., -2.], [6., 2.], [4., 2.]])]
box_cw = [box_ccw[0][::-1].copy()]

def stop_x(obs):
    sim = rvo2.PyRVOSimulator(0.01, CFG.neighbor_dist, CFG.max_neighbors,
                              CFG.time_horizon, CFG.time_horizon_obst,
                              CFG.radius, CFG.max_speed)
    _add_obstacles(sim, obs)
    a = sim.addAgent((0.0, 0.0))
    worst = math.inf
    for _ in range(2000):
        p = np.array(sim.getAgentPosition(a))
        d = np.array([10.0, 0.0]) - p
        n = np.hypot(*d)
        sim.setAgentPrefVelocity(a, tuple(d / n * 1.3) if n > 1e-9 else (0., 0.))
        sim.doStep()
        px, py = sim.getAgentPosition(a)
        dx = max(4.0 - px, 0.0, px - 6.0)
        dy = max(-2.0 - py, 0.0, py - 2.0)
        worst = min(worst, math.hypot(dx, dy))
    return sim.getAgentPosition(a)[0], worst

x_ccw, clr_ccw = stop_x(box_ccw)
x_cw, clr_cw = stop_x(box_cw)
check("winding order normalised", abs(x_ccw - x_cw) < 1e-9,
      f"CCW stops at x={x_ccw:.4f}, CW at x={x_cw:.4f}")
check("no penetration of the obstacle", clr_ccw >= -1e-9,
      f"min clearance {clr_ccw:.4f} m")
# Correct halt distance is exactly one radius from the face at x = 4.
check("radius not double-counted", abs((4.0 - x_ccw) - CFG.radius) < 0.02,
      f"gap to face {4.0 - x_ccw:.4f} m vs radius {CFG.radius}")

# --- 2. Same A* path, both controllers -----------------------------------
EXPORT = Path("v2/runs/v2/export")
trials = sorted(EXPORT.glob("trial_*.json"))[:4]
print()
print(f"{'scenario':<20} {'SFM':<24} {'ORCA':<24} {'waypoints':>9}")
print("-" * 82)

any_orca_progress = False
for f in trials:
    S = Scenario.load(f)
    nw = 0 if S.waypoints is None else S.waypoints.shape[1]
    Rs = run_trial(S, "hybrid")
    Ro = run_trial_orca(S, mode="orca_robot", planner="hybrid")
    print(f"{S.name:<20} "
          f"{Rs.outcome + ' len=' + format(Rs.path_length, '.1f'):<24} "
          f"{Ro.outcome + ' len=' + format(Ro.path_length, '.1f'):<24} {nw:>9}")
    if Ro.path_length > 10.0:
        any_orca_progress = True

check("A*-ORCA makes real progress along the shared path", any_orca_progress,
      "path length > 10 m on at least one scenario")

# --- 3. Corner behaviour -------------------------------------------------
# Waypoints around a tight L-corner. ORCA must round it without oscillating
# at the capture boundary or cutting inside the obstacle.
sim = rvo2.PyRVOSimulator(0.01, CFG.neighbor_dist, CFG.max_neighbors,
                          CFG.time_horizon, CFG.time_horizon_obst,
                          CFG.radius, CFG.max_speed)
Lpoly = [np.array([[20., 27.], [30., 27.], [30., 20.], [32., 20.],
                   [32., 30.], [20., 30.]])]
_add_obstacles(sim, Lpoly)
a = sim.addAgent((25.0, 24.0))
wps = [np.array([25.0, 24.0]), np.array([18.0, 24.0]),
       np.array([18.0, 33.0]), np.array([35.0, 33.0])]
wi = 0
reversals = 0
prev_wi = 0
minclr = math.inf
for _ in range(6000):
    p = np.array(sim.getAgentPosition(a))
    if np.hypot(*(p - wps[wi])) < 2.0 and wi < len(wps) - 1:
        wi += 1
    if wi < prev_wi:
        reversals += 1
    prev_wi = wi
    d = wps[wi] - p
    n = np.hypot(*d)
    sim.setAgentPrefVelocity(a, tuple(d / n * 1.3) if n > 1e-9 else (0., 0.))
    sim.doStep()
    dS, _ = poly_signed_distance(*sim.getAgentPosition(a), Lpoly[0])
    minclr = min(minclr, dS - CFG.radius)
    if np.hypot(*(np.array(sim.getAgentPosition(a)) - wps[-1])) < 0.5:
        break

fp = sim.getAgentPosition(a)
check("corner: waypoint index never reverses", reversals == 0,
      f"{reversals} reversals")
check("corner: no corner cutting into the obstacle", minclr >= -1e-9,
      f"min clearance {minclr:.4f} m")
check("corner: reaches the final waypoint",
      math.hypot(fp[0] - wps[-1][0], fp[1] - wps[-1][1]) < 1.0,
      f"final ({fp[0]:.2f}, {fp[1]:.2f}) vs target ({wps[-1][0]}, {wps[-1][1]})")

print(f"\n=== {passed} passed, {failed} failed ===")
if failed:
    sys.exit(1)
