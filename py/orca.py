"""ORCA local controller via the reference RVO2 library.

TWO DISTINCT COMPARISONS, kept separate because they answer different
questions and must not be conflated:

  ORCA_ROBOT ("orca_robot")
      The ROBOT plans with ORCA. Pedestrians remain SFM-driven exactly as in
      every other arm, and their states are pushed into the RVO2 simulator
      each step so ORCA sees their true motion. Pedestrian dynamics are
      therefore MATCHED across arms, which isolates the robot's local
      controller. This is the arm for the main method comparison.

  ORCA_CROWD ("orca_crowd")
      Every agent is an ORCA agent. This compares complete crowd MODELS, not
      robot controllers, and changes the pedestrians as well as the robot. It
      is a secondary ecosystem comparison and cannot be read as a controller
      comparison.

RECIPROCITY. ORCA assumes every agent takes a share of the avoidance effort.
In ORCA_ROBOT the pedestrians are not running ORCA, so that assumption is
violated: the robot plans as if they will yield half, and they will not.

There is no exact way to express "the robot takes all responsibility" through
the RVO2 public API, so this is an explicit experimental variable with three
candidate implementations, chosen on development scenarios rather than
assumed:

  'reciprocal'  Pedestrians are ORCA agents carrying their true SFM
                velocities. Faithful to the library, but the robot plans for
                only half the avoidance effort and the pedestrians supply
                none, so it UNDERSTATES what the robot must do.

  'frozen'      Pedestrian maxSpeed is set to 0 in the RVO2 view. Makes the
                robot do all the work, but it alters ORCA's MODEL of the
                pedestrian rather than reassigning responsibility, which can
                distort the relative-velocity geometry. Exploratory only.

  'margin'      Robot radius inflated by the buffer. Simple, but RVO2 uses
                one radius for BOTH agent and obstacle constraints, so this
                also makes the robot more conservative around STATIC geometry
                -- for which there is no reciprocity mismatch and therefore no
                justification. Retained only as a comparison.

  'ped_margin'  Pedestrian radii inflated by the buffer AS RVO2 SEES THEM,
                robot radius left true. In orca_robot mode the pedestrians are
                externally driven and RVO2 never integrates them, so their
                radius enters only the robot's agent-agent constraints. This
                confines the buffer to exactly the interaction where
                reciprocity fails, leaving obstacle conservatism untouched.
                PREFERRED.

None of these is exact ORCA-with-full-responsibility, and any write-up must
say so. The choice is frozen on development scenarios by the same lexicographic
objective used for the rest of the ORCA tuning.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import rvo2

from sfm import Params, Scenario, compute_forces


@dataclass
class OrcaConfig:
    """Fairness parameters. FROZEN before any comparative result is inspected.

    Chosen to match the SFM arms where a counterpart exists, so a difference
    in outcome cannot be attributed to a mismatched budget:
      radius        -> robot body radius
      max_speed     -> vMax, the same cap the SFM arms clamp to
      time_step     -> dt
      neighbor_dist -> rInfPed, the SFM pedestrian interaction cutoff
    Parameters with no SFM counterpart take RVO2 defaults and are tuned only
    on development scenarios, within the same tuning budget as the SFM arms.
    """
    neighbor_dist: float = 5.0
    max_neighbors: int = 10
    time_horizon: float = 2.0
    time_horizon_obst: float = 2.0
    radius: float = 0.35
    max_speed: float = 2.0
    # NAMING: the margin is a robot-side SAFETY BUFFER, not a responsibility
    # allocation. Inflating the robot radius changes the geometry of every
    # ORCA constraint; it compensates empirically for non-yielding pedestrians
    # but does not mathematically convert reciprocal half-responsibility into
    # full robot responsibility.
    responsibility: str = "ped_margin"    # 'reciprocal'|'frozen'|'margin'|'ped_margin'
    # SELECTION TOLERANCE, fixed here BEFORE the sweep runs: the smallest
    # buffer whose development safe-success is within this many percentage
    # points (absolute) of the best configuration wins. Choosing it after
    # seeing the sweep would let the threshold be fitted to a preferred answer.
    select_tolerance_pp: float = 5.0
    responsibility_margin: float = 0.20   # m, only used by 'margin'


def _add_obstacles(sim, obstacles):
    """Register polygons. RVO2 expects counter-clockwise vertex order."""
    for verts in obstacles:
        pts = [(float(v[0]), float(v[1])) for v in verts]
        area = 0.0
        for i in range(len(pts)):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % len(pts)]
            area += x1 * y2 - x2 * y1
        if area < 0:                      # clockwise -> reverse
            pts = pts[::-1]
        sim.addObstacle(pts)
    sim.processObstacles()


def run_trial_orca(S: Scenario, mode: str = "orca_robot",
                   cfg: OrcaConfig | None = None,
                   planner: str = "direct", log: bool = False,
                   aligned: bool = False):
    """Run one trial with the robot controlled by ORCA.

    mode     'orca_robot' (pedestrians stay SFM) or 'orca_crowd' (all ORCA)
    planner  'direct' (head for the goal) or 'hybrid' (follow A* waypoints)
    aligned  shared pre-update state, contact samples and waypoint timing
    """
    if aligned and mode != "orca_robot":
        raise ValueError("Aligned stepping requires orca_robot mode")
    if cfg is None:
        cfg = OrcaConfig()
    P = S.params
    nPed = S.ped_pos.shape[1]

    sim = rvo2.PyRVOSimulator(
        P.dt, cfg.neighbor_dist, cfg.max_neighbors,
        cfg.time_horizon, cfg.time_horizon_obst,
        cfg.radius, cfg.max_speed)
    _add_obstacles(sim, S.obstacles)

    # 'margin' inflates only the ROBOT's radius, leaving every pedestrian
    # model untouched.
    # FAIRNESS INVARIANT: r_eff is a CONTROL parameter only. Every collision
    # and clearance OUTCOME below is measured with the true body radius
    # P.rRobot. Scoring ORCA against its own inflated radius would credit it
    # with clearance it does not physically have, and would make the safety
    # comparison meaningless.
    r_eff = cfg.radius
    ped_r_eff = P.rPed
    if mode == "orca_robot":
        if cfg.responsibility == "margin":
            r_eff = cfg.radius + cfg.responsibility_margin
        elif cfg.responsibility == "ped_margin":
            # Buffer confined to agent-agent constraints: obstacle handling
            # keeps the true robot radius.
            ped_r_eff = P.rPed + cfg.responsibility_margin
    rid = sim.addAgent((float(S.robot_start[0]), float(S.robot_start[1])),
                       cfg.neighbor_dist, cfg.max_neighbors,
                       cfg.time_horizon, cfg.time_horizon_obst,
                       r_eff, cfg.max_speed,
                       (float(S.robot_vel[0]), float(S.robot_vel[1])))
    # RVO2 requires either position alone or the complete parameter list
    # including initial velocity; a partial list is rejected.
    pids = [sim.addAgent((float(S.ped_pos[0, i]), float(S.ped_pos[1, i])),
                         cfg.neighbor_dist, cfg.max_neighbors,
                         cfg.time_horizon, cfg.time_horizon_obst,
                         ped_r_eff, P.v0Ped,
                         (float(S.ped_vel[0, i]), float(S.ped_vel[1, i])))
            for i in range(nPed)]

    if mode == "orca_robot" and cfg.responsibility == "frozen":
        for pid in pids:
            sim.setAgentMaxSpeed(pid, 0.0)

    # Shadow SFM state for the pedestrians in orca_robot mode.
    x = np.zeros((2, 1 + nPed))
    v = np.zeros((2, 1 + nPed))
    x[:, 0] = S.robot_start
    v[:, 0] = S.robot_vel
    if nPed:
        x[:, 1:] = S.ped_pos
        v[:, 1:] = S.ped_vel
    radii = np.array([P.rRobot] + [P.rPed] * nPed)
    v0 = np.array([P.v0Robot] + [P.v0Ped] * nPed)
    goals = np.zeros((2, 1 + nPed))
    if nPed:
        goals[:, 1:] = S.ped_goal

    if planner == "hybrid" and S.waypoints is not None and S.waypoints.shape[1]:
        wi = 0
    else:
        planner, wi = "direct", 0

    pool_idx = 0
    dwell = stall = 0
    path_length = 0.0
    ped_col = obs_col = 0
    in_ped = in_obs = False
    min_pg = min_og = math.inf
    outcome, steps_taken = "timeout", P.maxSteps
    traj = np.zeros((2, P.maxSteps)) if log else None

    for step in range(1, P.maxSteps + 1):
        # Aligned mode uses one shared pre-update snapshot for both controllers.
        if aligned:
            F, mpg, mog = compute_forces(x, v, goals, radii, v0, S.obstacles, P)
            for i, pid in enumerate(pids):
                sim.setAgentPosition(pid, tuple(x[:, i + 1]))
                sim.setAgentVelocity(pid, tuple(v[:, i + 1]))
        # --- pedestrians: SFM in orca_robot, ORCA in orca_crowd -----------
        if mode == "orca_robot" and nPed:
            if not aligned:
                F, mpg, mog = compute_forces(x, v, goals, radii, v0, S.obstacles, P)
            v[:, 1:] = v[:, 1:] + F[:, 1:] * P.dt
            sp = np.hypot(v[0, 1:], v[1, 1:])
            over = sp > P.vMax
            if over.any():
                v[:, 1:][:, over] = v[:, 1:][:, over] / sp[over] * P.vMax
            if not aligned:
                x[:, 1:] = x[:, 1:] + v[:, 1:] * P.dt
                for i, pid in enumerate(pids):
                    sim.setAgentPosition(pid, (float(x[0, i + 1]), float(x[1, i + 1])))
                    sim.setAgentVelocity(pid, (float(v[0, i + 1]), float(v[1, i + 1])))
        elif nPed:
            for i, pid in enumerate(pids):
                gp = goals[:, i + 1] - np.array(sim.getAgentPosition(pid))
                n = np.hypot(*gp)
                pref = (0.0, 0.0) if n < 1e-9 else tuple(gp / n * P.v0Ped)
                sim.setAgentPrefVelocity(pid, pref)

        # --- robot preferred velocity -------------------------------------
        rpos = np.array(sim.getAgentPosition(rid))
        if planner == "hybrid":
            if (not aligned and np.hypot(*(rpos - S.waypoints[:, wi])) < P.rWaypoint
                    and wi < S.waypoints.shape[1] - 1):
                wi += 1
            target = S.waypoints[:, wi]
        else:
            target = np.asarray(P.goal)
        dg = target - rpos
        dist = float(np.hypot(*dg))
        # Same tapered desired speed the SFM arms use, so arrival behaviour is
        # comparable rather than an artefact of a different stopping rule.
        vdes = P.v0Robot * min(1.0, float(np.hypot(*(np.asarray(P.goal) - rpos))) / P.dSlow)
        pref = (0.0, 0.0) if dist < 1e-9 else tuple(dg / dist * vdes)
        sim.setAgentPrefVelocity(rid, pref)

        sim.doStep()
        if aligned:
            x[:, 1:] = x[:, 1:] + v[:, 1:] * P.dt

        new_rpos = np.array(sim.getAgentPosition(rid))
        rvel = np.array(sim.getAgentVelocity(rid))
        path_length += float(np.hypot(*(new_rpos - rpos)))
        x[:, 0] = new_rpos
        v[:, 0] = rvel
        if mode == "orca_crowd" and nPed:
            for i, pid in enumerate(pids):
                x[:, i + 1] = sim.getAgentPosition(pid)
                v[:, i + 1] = sim.getAgentVelocity(pid)

        if log:
            traj[:, step - 1] = new_rpos

        # Score the same pre-update gaps as the SFM controller in aligned mode.
        if aligned:
            gp, go = float(mpg[0]), float(mog[0])
        else:
            if nPed:
                gaps = np.hypot(x[0, 1:] - x[0, 0], x[1, 1:] - x[1, 0]) - (P.rRobot + P.rPed)
                gp = float(gaps.min())
            else:
                gp = math.inf
            go = math.inf
            for verts in S.obstacles:
                from sfm import poly_signed_distance
                dS, _ = poly_signed_distance(x[0, 0], x[1, 0], verts)
                go = min(go, dS - P.rRobot)
        min_pg, min_og = min(min_pg, gp), min(min_og, go)
        if gp < 0:
            if not in_ped:
                ped_col += 1; in_ped = True
        else:
            in_ped = False
        if go < 0:
            if not in_obs:
                obs_col += 1; in_obs = True
        else:
            in_obs = False

        if aligned and planner == "hybrid":
            if (np.hypot(*(x[:, 0] - S.waypoints[:, wi])) < P.rWaypoint
                    and wi < S.waypoints.shape[1] - 1):
                wi += 1

        # --- pedestrian goal pool ------------------------------------------
        for i in range(1, 1 + nPed):
            if np.hypot(*(x[:, i] - goals[:, i])) < P.pedGoalTol:
                if pool_idx >= S.goal_pool.shape[1]:
                    raise RuntimeError(f"goal pool exhausted at step {step}")
                goals[:, i] = S.goal_pool[:, pool_idx]
                pool_idx += 1

        # --- termination, identical rule to the SFM arms --------------------
        dist_goal = float(np.hypot(*(x[:, 0] - np.asarray(P.goal))))
        rob_speed = float(np.hypot(*v[:, 0]))
        if dist_goal <= P.rGoal and rob_speed <= P.vGoal and gp >= 0:
            dwell += 1
            if dwell >= P.dwellSteps:
                outcome, steps_taken = "success", step
                break
        else:
            dwell = 0
        if rob_speed < P.stallSpeed and dist_goal > P.rGoal:
            stall += 1
            if stall >= P.stallSteps:
                outcome, steps_taken = "deadlock", step
                break
        else:
            stall = 0

    from sfm import Result
    R = Result()
    R.outcome = outcome
    R.steps = steps_taken
    R.path_length = path_length
    R.final_dist = float(np.hypot(*(x[:, 0] - np.asarray(P.goal))))
    R.final_speed = float(np.hypot(*v[:, 0]))
    R.ped_collisions = ped_col
    R.obs_collisions = obs_col
    R.min_ped_gap = min_pg
    R.min_obs_gap = min_og
    if log:
        R.traj = traj[:, :steps_taken]
    return R
