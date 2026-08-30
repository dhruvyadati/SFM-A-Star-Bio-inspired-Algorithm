"""Randomized differential-test fixtures for MATLAB vs Python.

Hand-built fixtures test the edge cases someone thought of. Every bug found so
far lived in a code path no fixture happened to exercise -- zero pedestrians,
a single agent row, an array-valued per-pedestrian threshold. Randomised
scenarios cover interactions nobody anticipated.

Generated HERE rather than in MATLAB so no A* or graph machinery is involved:
the target is the shared dynamics, and a difference in scenario construction
would be indistinguishable from a difference in integration.

Deliberately small (few agents, few obstacles, short horizon) so a full
differential sweep is minutes rather than hours, and deliberately varied in
agent count including the degenerate 0 and 1 cases.
"""
import json
import sys
from pathlib import Path

import numpy as np

OUT = Path("v2/runs/v2/export_rand")
OUT.mkdir(parents=True, exist_ok=True)

N_CASES = int(sys.argv[1]) if len(sys.argv) > 1 else 24
rng = np.random.default_rng(90210)

PARAMS = {
    "dt": 0.01, "tau": 0.5, "v0Robot": 1.30, "v0Ped": 1.34, "vMax": 2.00,
    "dSlow": 2.0, "aPed": 2.10, "aPedRobot": [], "bPed": 0.30, "rInfPed": 5.0,
    "aObs": 10.0, "bObs": 0.20, "rInfObs": 3.0, "kBody": 200.0,
    "rRobot": 0.35, "rPed": 0.30, "goal": [50.0, 50.0], "start": [1.0, 1.0],
    "xMin": 1.0, "xMax": 50.0, "yMin": 1.0, "yMax": 50.0,
    "maxSteps": 2500,          # short: the target is agreement, not outcomes
    "rGoal": 0.5, "vGoal": 0.10, "dwellSteps": 100,
    "stallSpeed": 0.05, "stallSteps": 500, "pedGoalTol": 1.0,
    "rWaypoint": 2.0, "graphMargin": 1.0,
    "ttcEnabled": False, "ttcHorizon": 2.0, "ttcStop": 0.8,
    "ttcClearance": 0.15, "ttcMinGap": 0.40, "ttcHardGap": 0.12,
    "ttcRecedeBand": 0.05, "ttcSlewRate": 4.0,
    "zoneIntimate": 0.45, "zonePersonal": 1.20, "zoneNearCollision": 0.10,
    "obsNearClearance": 0.15, "typeNegligible": 0.05, "typeLookback": 100,
    "numPedestrians": 0, "numObstacles": 0, "maxArea": 150,
    "vertexRange": [3, 10], "cellSize": 1.0,
}


def random_convex_polygon(cx, cy, r, nv, rng):
    """Convex polygon by sorted angles -- guarantees no self-intersection."""
    ang = np.sort(rng.uniform(0, 2 * np.pi, nv))
    rad = r * rng.uniform(0.5, 1.0, nv)
    return np.stack([cx + rad * np.cos(ang), cy + rad * np.sin(ang)], axis=1)


written = 0
for i in range(N_CASES):
    # Agent counts span the degenerate cases as well as the crowded ones.
    n_ped = int(rng.choice([0, 1, 2, 5, 12, 25]))
    n_obs = int(rng.choice([0, 1, 2, 4]))

    obstacles = []
    for _ in range(n_obs):
        cx, cy = rng.uniform(12, 40, 2)
        obstacles.append(random_convex_polygon(cx, cy, rng.uniform(2.0, 5.0),
                                               int(rng.integers(3, 9)), rng).tolist())

    def free_point():
        for _ in range(500):
            p = rng.uniform(3, 47, 2)
            ok = True
            for poly in obstacles:
                a = np.asarray(poly)
                if (np.min(np.hypot(a[:, 0] - p[0], a[:, 1] - p[1])) < 2.0):
                    ok = False
                    break
            if ok:
                return p
        return rng.uniform(3, 47, 2)

    ped_pos = [free_point().tolist() for _ in range(n_ped)]
    ped_vel = rng.normal(0, 0.6, (n_ped, 2)).tolist() if n_ped else []
    ped_goal = [free_point().tolist() for _ in range(n_ped)]
    pool = [free_point().tolist() for _ in range(400)]

    rob = free_point()
    case = {
        "name": f"rand{i:03d}",
        "params": dict(PARAMS, numPedestrians=n_ped, numObstacles=n_obs),
        "obstacles": obstacles,
        "robotStart": rob.tolist(),
        "robotVel": rng.normal(0, 0.5, 2).tolist(),
        "pedPos": ped_pos, "pedVel": ped_vel, "pedGoal": ped_goal,
        "pedGoalSeed": 0, "goalPool": pool, "waypoints": [],
        "condition": f"rand_p{n_ped}_o{n_obs}", "matlabOutcome": "",
    }
    (OUT / f"rand_{case['name']}.json").write_text(json.dumps(case, indent=1))
    written += 1

print(f"wrote {written} randomized fixtures to {OUT}")
counts = {}
for f in sorted(OUT.glob("*.json")):
    c = json.loads(f.read_text())
    counts[c["condition"]] = counts.get(c["condition"], 0) + 1
print("agent/obstacle combinations covered:")
for k in sorted(counts):
    print(f"  {k:<16} x{counts[k]}")
