"""Reciprocal (ORCA) pedestrians against the unchanged robot controller.

Amendment E. Tests whether the confirmed exposure mechanism depends on
pedestrians that do not yield.

WHAT VARIES AND WHAT DOES NOT. The robot keeps its exact SFM + A* + TTC
controller, integrator and instrumentation. Only the PEDESTRIAN model changes.
Driving the robot with ORCA as well would change the intervention under test at
the same time as the pedestrian model, and no comparison could separate them.

SFM pedestrians feel a soft repulsive force that is provably too weak to
prevent contact: the robot's goal drive v0/tau = 2.60 m/s^2 exceeds pedestrian
repulsion at contact (aPed = 2.10). They are pushed through rather than
stepping aside. ORCA pedestrians instead solve for a collision-free velocity
each step, so they actively sidestep. That is the qualitative change under test.

RESPONSIBILITY, stated honestly. The robot does not run ORCA, so it does not
take its half of the reciprocal avoidance. Standard ORCA (50/50) therefore
gives pedestrians HALF the avoidance effort of a true ORCA-ORCA pair. There is
no exact way to express "pedestrians take all responsibility" in RVO2; the
`ped_yield` mode approximates it by inflating the robot's radius as seen by
pedestrians, which increases clearance without mathematically reassigning
responsibility. Both are reported, and neither is described as exact.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rvo2

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Scenario, run_trial


@dataclass
class PedOrcaConfig:
    neighbor_dist: float = 5.0
    max_neighbors: int = 10
    time_horizon: float = 2.0
    time_horizon_obst: float = 1.0
    # 'reciprocal' = standard ORCA 50/50 (robot does not reciprocate)
    # 'ped_yield'  = robot radius inflated in the pedestrians' view
    responsibility: str = "reciprocal"
    robot_inflation: float = 0.20      # m, only used by 'ped_yield'


def _add_obstacles(sim, obstacles):
    for verts in obstacles:
        v = [(float(a), float(b)) for a, b in zip(verts[:, 0], verts[:, 1])]
        # RVO2 obstacles must be counter-clockwise to be solid from outside.
        area = sum(v[i][0] * v[(i + 1) % len(v)][1] - v[(i + 1) % len(v)][0] * v[i][1]
                   for i in range(len(v)))
        sim.addObstacle(v if area > 0 else v[::-1])
    sim.processObstacles()


class OrcaPedestrians:
    """Callback driving pedestrian velocities with ORCA each step."""

    def __init__(self, S: Scenario, cfg: PedOrcaConfig):
        P = S.params
        self.P, self.cfg = P, cfg
        self.n = S.ped_pos.shape[1]
        self.sim = rvo2.PyRVOSimulator(
            P.dt, cfg.neighbor_dist, cfg.max_neighbors,
            cfg.time_horizon, cfg.time_horizon_obst, P.rPed, P.vMax)
        _add_obstacles(self.sim, S.obstacles)
        self.ped_ids = [
            self.sim.addAgent((float(S.ped_pos[0, i]), float(S.ped_pos[1, i])),
                              cfg.neighbor_dist, cfg.max_neighbors,
                              cfg.time_horizon, cfg.time_horizon_obst,
                              P.rPed, P.vMax,
                              (float(S.ped_vel[0, i]), float(S.ped_vel[1, i])))
            for i in range(self.n)]
        r_rob = P.rRobot + (cfg.robot_inflation
                            if cfg.responsibility == "ped_yield" else 0.0)
        self.robot_id = self.sim.addAgent(
            (float(S.robot_start[0]), float(S.robot_start[1])),
            cfg.neighbor_dist, cfg.max_neighbors, cfg.time_horizon,
            cfg.time_horizon_obst, r_rob, P.vMax, (0.0, 0.0))

    def __call__(self, step, x, v, radii, goals):
        P = self.P
        # The robot is mirrored into the ORCA world as a moving obstacle whose
        # state is dictated by its own controller, never by ORCA.
        self.sim.setAgentPosition(self.robot_id, (float(x[0, 0]), float(x[1, 0])))
        self.sim.setAgentVelocity(self.robot_id, (float(v[0, 0]), float(v[1, 0])))
        self.sim.setAgentPrefVelocity(self.robot_id, (float(v[0, 0]), float(v[1, 0])))

        for i, aid in enumerate(self.ped_ids):
            self.sim.setAgentPosition(aid, (float(x[0, i + 1]), float(x[1, i + 1])))
            self.sim.setAgentVelocity(aid, (float(v[0, i + 1]), float(v[1, i + 1])))
            # Preferred velocity toward the goal the SFM engine maintains, so
            # goal sequencing is identical across pedestrian models.
            d = goals[:, i + 1] - x[:, i + 1]
            n = float(np.hypot(d[0], d[1]))
            pref = (0.0, 0.0) if n < 1e-9 else (
                float(d[0] / n * P.v0Ped), float(d[1] / n * P.v0Ped))
            self.sim.setAgentPrefVelocity(aid, pref)

        self.sim.doStep()
        out = np.empty((2, self.n))
        for i, aid in enumerate(self.ped_ids):
            vx, vy = self.sim.getAgentVelocity(aid)
            out[0, i], out[1, i] = vx, vy
        return out


def run_trial_reciprocal(S: Scenario, planner: str = "hybrid", ttc: bool = False,
                         cfg: PedOrcaConfig | None = None, log: bool = False):
    return run_trial(S, planner, log=log, ttc=ttc,
                     ped_velocity_fn=OrcaPedestrians(S, cfg or PedOrcaConfig()))
