"""Amendment H: the frozen 960-run external transfer study.

    primary               4 strata x 50 seeds x 2 arms, visible=false, dt=0.25  = 400
    visibility sensitivity  same, visible=true                                  = 400
    timestep sensitivity  4 strata x seeds 0-9 x 2 arms x 2 visibilities, dt=0.01 = 160

Arms are DIRECT-GOAL SFM and DIRECT-GOAL SFM-TTC. Not "guided": CrowdNav's
native scenarios contain no static obstacle-routing problem, so the A* route
degenerates to a straight start-to-goal reference. This study tests transfer of
the TTC safety-mobility effect under an external crowd simulator, NOT transfer
of the guidance x TTC interaction.

Pairing is on initial state and random stream (`test_case`), not on identical
realised pedestrian trajectories -- CrowdNav humans react to the robot, so
trajectories legitimately diverge once the arms behave differently.

CrowdNav `Timeout` and our own 5 s low-speed deadlock are recorded SEPARATELY.
A timeout means the episode horizon expired; it does not establish that the
robot met the frozen deadlock criterion.
"""
from __future__ import annotations

import configparser
import json
import sys
from itertools import product
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from crowdnav_adapter import CROWDNAV, SfmTtcPolicy

sys.path.insert(0, str(CROWDNAV))
from crowd_sim.envs.crowd_sim import CrowdSim          # noqa: E402
from crowd_sim.envs.utils.robot import Robot           # noqa: E402

OUT = Path("v2/runs/v2/crowdnav_transfer.json")
STRATA = [(r, n) for r in ("circle_crossing", "square_crossing") for n in (5, 10)]
SEEDS = list(range(50))
SENS_SEEDS = list(range(10))       # prespecified subset, fixed before any run
# HORIZON IN SECONDS, never a step count. `MAX_STEPS = 400` would be 100 s at
# dt = 0.25 but only 4 s at dt = 0.01 -- far too short to cross the environment,
# so every timestep-sensitivity run would have "timed out" as an artefact of
# the units. CrowdNav's OWN time_limit (25 s) governs the `Timeout` outcome;
# this is only a backstop against a hang and should never bind.
HORIZON_SECONDS = 100.0
WORKERS = 10


def one(job):
    rule, n_human, seed, ttc, visible, dt = job
    cfg = configparser.RawConfigParser()
    cfg.read(str(CROWDNAV / "crowd_nav/configs/env.config"))
    cfg.set("sim", "human_num", str(n_human))
    cfg.set("sim", "test_sim", rule)
    cfg.set("robot", "visible", "true" if visible else "false")
    cfg.set("env", "time_step", str(dt))

    env = CrowdSim()
    env.configure(cfg)
    pol = SfmTtcPolicy(ttc=ttc, dt=dt)
    pol.time_step = dt
    rob = Robot(cfg, "robot")
    rob.set_policy(pol)
    rob.time_step = dt
    env.set_robot(rob)

    max_steps = HORIZON_SECONDS / dt
    assert abs(max_steps - round(max_steps)) < 1e-9, (
        f"horizon {HORIZON_SECONDS}s is not a whole number of {dt}s steps")
    max_steps = int(round(max_steps))

    ob = env.reset(phase="test", test_case=seed)
    done, info, k = False, None, 0
    min_dist = float("inf")
    while not done and k < max_steps:
        ob, _, done, info = env.step(rob.act(ob))
        d = min((float(np.hypot(h.px - rob.px, h.py - rob.py)) - h.radius - rob.radius)
                for h in ob) if ob else float("inf")
        min_dist = min(min_dist, d)
        k += 1

    outcome = type(info).__name__
    tr = pol.trace
    return dict(
        rule=rule, n_human=n_human, seed=seed, arm="ttc" if ttc else "reference",
        visible=visible, dt=dt,
        outcome=outcome,
        reach_goal=outcome == "ReachGoal",
        collision=outcome == "Collision",
        timeout=outcome == "Timeout" or k >= max_steps,
        deadlock=bool(pol.deadlock),            # OUR criterion, not CrowdNav's
        steps=k, elapsed_s=k * dt,
        min_clearance=float(min_dist),
        ttc_active_steps=sum(1 for e in tr if e["ttc_active"]),
        mandatory_steps=sum(1 for e in tr if e["mandatory"]),
        slew_limited_steps=sum(1 for e in tr if e["slew_limited"]),
        clipped_steps=int(pol.n_clipped),
        clipped_frac=pol.n_clipped / max(len(tr), 1),
        max_requested_speed=float(pol.max_requested),
        mean_applied_speed=float(np.mean([e["applied_speed"] for e in tr])) if tr else 0.0,
    )


def jobs():
    out = []
    for (rule, n), seed, ttc in product(STRATA, SEEDS, (False, True)):
        out.append((rule, n, seed, ttc, False, 0.25))      # primary
        out.append((rule, n, seed, ttc, True, 0.25))       # visibility sensitivity
    for (rule, n), seed, ttc, vis in product(STRATA, SENS_SEEDS, (False, True), (False, True)):
        out.append((rule, n, seed, ttc, vis, 0.01))        # timestep sensitivity
    return out


if __name__ == "__main__":
    J = jobs()
    print(f"strata {len(STRATA)}  seeds {len(SEEDS)}  jobs {len(J)}", flush=True)
    assert len(J) == 960, f"expected 960 runs, built {len(J)}"
    rows = []
    with Pool(WORKERS) as pool:
        for i, r in enumerate(pool.imap_unordered(one, J, chunksize=4), 1):
            rows.append(r)
            if i % 100 == 0:
                print(f"  {i}/{len(J)}", flush=True)
    OUT.write_text(json.dumps(rows))
    print(f"wrote {OUT}  n={len(rows)}", flush=True)
