"""EXPLORATORY COPY of sfm.py — do NOT use for confirmatory results.

Adds `unsafe_event_step`, which Amendment A.4b defined but which was never
implemented in the confirmatory executable. Kept in a separate file so
py/sfm.py stays byte-identical to the hash recorded before the confirmatory
run, preserving that audit trail.

Used only for deterministic replay of already-analysed scenarios.

Original header follows.

Corrected Social Force Model, Python port.

A deliberate re-implementation from the same specification as the MATLAB
version, not a transliteration. Independent implementations that agree are
evidence the dynamics are right; a transliteration that agrees only proves the
translation worked.

Conventions that must match MATLAB exactly, and are asserted by the
cross-validation harness:

  * unit mass, so every force term is an acceleration in m/s^2
  * semi-implicit (symplectic) Euler: velocity FIRST, then position using the
    UPDATED velocity
  * speed clamped to vMax after the force update, no lower clamp
  * goal desired-speed taper: v0 * min(1, d / dSlow)
  * repulsion uses UNIT direction vectors and decays monotonically
  * obstacle signed distance is NEGATIVE inside a polygon, and a penetrating
    agent is pushed outward along the boundary normal
  * pedestrian goals consumed IN ORDER from the scenario's goal pool
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
# Geometry


def poly_signed_distance(px: float, py: float, verts: np.ndarray):
    """Signed distance from a point to a polygon, negative inside.

    Mirrors p_poly_dist: distance to the nearest edge, sign from containment.
    Returns (distance, closest_point).
    """
    n = len(verts)
    best_d2 = math.inf
    best_pt = (verts[0, 0], verts[0, 1])

    for i in range(n):
        ax, ay = verts[i]
        bx, by = verts[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        seg2 = ex * ex + ey * ey
        if seg2 < 1e-18:
            t = 0.0
        else:
            t = ((px - ax) * ex + (py - ay) * ey) / seg2
            t = max(0.0, min(1.0, t))
        cx, cy = ax + t * ex, ay + t * ey
        d2 = (px - cx) ** 2 + (py - cy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_pt = (cx, cy)

    d = math.sqrt(best_d2)
    return (-d if point_in_poly(px, py, verts) else d), best_pt


def poly_signed_distance_many(pts: np.ndarray, verts: np.ndarray):
    """Signed distance from MANY points to one polygon, vectorised.

    pts is (2, M). Returns (dist (M,), closest (2, M)), negative inside.

    The scalar version is called once per agent per obstacle per step, which
    for 49 agents and 10 obstacles is ~490 Python-level calls every 0.01 s of
    simulated time. Vectorising over agents turns that into one call per
    obstacle and is what makes the port faster than the MATLAB original rather
    than slower.
    """
    px, py = pts[0], pts[1]
    n = len(verts)
    ax = verts[:, 0][:, None]
    ay = verts[:, 1][:, None]
    bx = np.roll(verts[:, 0], -1)[:, None]
    by = np.roll(verts[:, 1], -1)[:, None]

    ex, ey = bx - ax, by - ay
    seg2 = ex * ex + ey * ey
    seg2 = np.where(seg2 < 1e-18, 1.0, seg2)
    tt = ((px[None, :] - ax) * ex + (py[None, :] - ay) * ey) / seg2
    tt = np.clip(tt, 0.0, 1.0)
    cx = ax + tt * ex
    cy = ay + tt * ey
    d2 = (px[None, :] - cx) ** 2 + (py[None, :] - cy) ** 2

    k = np.argmin(d2, axis=0)
    cols = np.arange(pts.shape[1])
    dist = np.sqrt(d2[k, cols])
    closest = np.vstack([cx[k, cols], cy[k, cols]])

    # Ray casting, vectorised over points and edges.
    yi, yj = verts[:, 1][:, None], np.roll(verts[:, 1], -1)[:, None]
    xi, xj = verts[:, 0][:, None], np.roll(verts[:, 0], -1)[:, None]
    cond = (yi > py[None, :]) != (yj > py[None, :])
    denom = np.where(np.abs(yj - yi) < 1e-18, 1.0, yj - yi)
    xint = (xj - xi) * (py[None, :] - yi) / denom + xi
    crossings = np.sum(cond & (px[None, :] < xint), axis=0)
    inside = (crossings % 2) == 1

    return np.where(inside, -dist, dist), closest


def point_in_poly(px: float, py: float, verts: np.ndarray) -> bool:
    """Ray-casting containment test."""
    inside = False
    n = len(verts)
    j = n - 1
    for i in range(n):
        xi, yi = verts[i]
        xj, yj = verts[j]
        if (yi > py) != (yj > py):
            xint = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < xint:
                inside = not inside
        j = i
    return inside


# ---------------------------------------------------------------------------
# Parameters


@dataclass
class Params:
    dt: float = 0.01
    tau: float = 0.5
    v0Robot: float = 1.30
    v0Ped: float = 1.34
    vMax: float = 2.00
    dSlow: float = 2.0
    aPed: float = 2.10
    aPedRobot: float | None = None
    bPed: float = 0.30
    rInfPed: float = 5.0
    aObs: float = 10.0
    bObs: float = 0.20
    rInfObs: float = 3.0
    kBody: float = 200.0
    rRobot: float = 0.35
    rPed: float = 0.30
    goal: tuple = (50.0, 50.0)
    start: tuple = (1.0, 1.0)
    xMin: float = 1.0
    xMax: float = 50.0
    yMin: float = 1.0
    yMax: float = 50.0
    maxSteps: int = 15000
    rGoal: float = 0.5
    vGoal: float = 0.10
    dwellSteps: int = 100
    stallSpeed: float = 0.05
    stallSteps: int = 500
    pedGoalTol: float = 1.0
    rWaypoint: float = 2.0
    graphMargin: float = 1.0
    # Time-to-collision filter. Disabled by default so the unfiltered
    # controller stays measurable as its own arm.
    ttcEnabled: bool = False
    ttcHorizon: float = 2.0
    ttcStop: float = 0.8
    ttcClearance: float = 0.15
    ttcMinGap: float = 0.40
    ttcHardGap: float = 0.12
    ttcRecedeBand: float = 0.05
    ttcSlewRate: float = 4.0
    aPedRobotOverride: float | None = None

    @classmethod
    def from_json(cls, d: dict) -> "Params":
        f = {k.name for k in cls.__dataclass_fields__.values()}
        kw = {}
        for k, v in d.items():
            if k not in f:
                continue
            # MATLAB writes an unset optional scalar as [], which must map to
            # None BEFORE any list-to-tuple conversion, or it becomes an empty
            # tuple and silently breaks the broadcast at use time.
            if k == "aPedRobot":
                kw[k] = None if (v is None or v == [] or v == ()) else float(v)
                continue
            if isinstance(v, list):
                v = tuple(v)
            kw[k] = v
        return cls(**kw)


# ---------------------------------------------------------------------------
# Forces


def compute_forces(x, v, goals, radii, v0, obstacles, P: Params):
    """Net force on every agent. x, v, goals are (2, N). Returns (2, N)."""
    N = x.shape[1]
    F = np.zeros((2, N))

    # --- goal attraction, unit direction, tapered desired speed -----------
    dg = goals - x
    dist = np.hypot(dg[0], dg[1])
    nz = dist > 1e-12
    ehat = np.zeros((2, N))
    ehat[:, nz] = dg[:, nz] / dist[nz]
    vDes = v0 * np.minimum(1.0, dist / P.dSlow)
    F += (ehat * vDes - v) / P.tau

    # --- agent repulsion, monotonically decreasing, maximal at contact ----
    dx = x[0][:, None] - x[0][None, :]
    dy = x[1][:, None] - x[1][None, :]
    d = np.hypot(dx, dy)
    rSum = radii[:, None] + radii[None, :]
    eye = np.eye(N, dtype=bool)

    gap = d - rSum
    gap[eye] = np.inf
    min_ped_gap = gap.min(axis=1)

    act = (~eye) & (d <= P.rInfPed) & (d > 1e-12)
    if act.any():
        # Row 1..N is the RECEIVING agent. Only row 0 (the robot) carries the
        # robot's own gain, so pedestrian-pedestrian dynamics are unchanged by
        # a robot-gain sweep and arms keep a common crowd model.
        aMat = np.full((N, N), P.aPed)
        if P.aPedRobot is not None:
            aMat[0, :] = P.aPedRobot
        dSafe = np.where(act, d, 1.0)
        mag = aMat * np.exp((rSum - d) / P.bPed)
        mag = mag + P.kBody * np.maximum(0.0, rSum - d)  # contact, symmetric
        mag = np.where(act, mag, 0.0)
        F[0] += (mag * dx / dSafe).sum(axis=1)
        F[1] += (mag * dy / dSafe).sum(axis=1)

    # --- static obstacle repulsion, vectorised over agents ----------------
    min_obs_gap = np.full(N, np.inf)
    for verts in obstacles:
        dS, closest = poly_signed_distance_many(x, verts)
        to_surf = closest - x
        ns = np.hypot(to_surf[0], to_surf[1])
        safe = np.where(ns > 1e-12, ns, 1.0)

        inside = dS < 0.0
        # Outward normal: from the boundary toward the agent when outside,
        # toward the nearest boundary point when inside, so a penetrating
        # agent is always driven out rather than deeper in.
        nhat = np.where(inside, to_surf / safe, -to_surf / safe)
        degenerate = ns <= 1e-12
        if degenerate.any():
            nhat[0, degenerate] = 1.0
            nhat[1, degenerate] = 0.0

        mag_in = P.aObs * np.exp(radii / P.bObs) + P.kBody * (radii + ns)
        mag_out = P.aObs * np.exp((radii - dS) / P.bObs)
        mag_out = mag_out + P.kBody * np.maximum(0.0, radii - dS)
        active = inside | (dS <= P.rInfObs)
        mag = np.where(inside, mag_in, mag_out) * active

        F += nhat * mag
        min_obs_gap = np.minimum(min_obs_gap, dS - radii)

    return F, min_ped_gap, min_obs_gap


# ---------------------------------------------------------------------------
# Scenario and simulation


@dataclass
class Scenario:
    name: str
    params: Params
    obstacles: list
    robot_start: np.ndarray
    robot_vel: np.ndarray
    ped_pos: np.ndarray      # (2, nPed)
    ped_vel: np.ndarray
    ped_goal: np.ndarray
    goal_pool: np.ndarray    # (2, nPool)
    waypoints: np.ndarray | None
    matlab_outcome: str = ""
    condition: str = ""
    # {pedestrian index -> hold seconds}. Defaults to None, so every
    # scenario generated before Amendment G is unaffected.
    blocker_hold: dict | None = None
    # Amendment G2: {"t": seconds, "obstacles": [poly, ...]} -- a static map
    # change at a frozen time, identical for every arm. Absent by default.
    map_change: dict | None = None

    @classmethod
    def load(cls, path) -> "Scenario":
        d = json.loads(Path(path).read_text())

        def arr(key):
            a = np.asarray(d.get(key) or [], dtype=float)
            if a.size == 0:
                return np.zeros((2, 0))
            if a.ndim == 1:
                a = a[None, :]
            return a.T                      # file is (n, 2); internal is (2, n)

        obs = [np.asarray(o, dtype=float) for o in (d.get("obstacles") or [])]
        wp = d.get("waypoints")
        wpa = None
        if wp:
            wpa = np.asarray(wp, dtype=float)
            if wpa.ndim == 1:
                wpa = wpa[None, :]
            wpa = wpa.T

        return cls(
            name=d["name"],
            params=Params.from_json(d["params"]),
            obstacles=obs,
            robot_start=np.asarray(d["robotStart"], dtype=float),
            robot_vel=np.asarray(d["robotVel"], dtype=float),
            ped_pos=arr("pedPos"),
            ped_vel=arr("pedVel"),
            ped_goal=arr("pedGoal"),
            goal_pool=arr("goalPool"),
            # {pedestrian index -> hold seconds}. Absent in every scenario
            # generated before Amendment G, so the field defaults to None and
            # the engine path is unchanged.
            blocker_hold=({int(i): float(h) for i, h in d["blockerHold"]}
                          if d.get("blockerHold") else None),
            map_change=({"t": float(d["mapChange"]["t"]),
                         "obstacles": [np.asarray(o, float)
                                       for o in d["mapChange"]["obstacles"]]}
                        if d.get("mapChange") else None),
            waypoints=wpa,
            matlab_outcome=d.get("matlabOutcome", ""),
            condition=d.get("condition", ""),
        )


@dataclass
class Result:
    # Mediator instrumentation. Recorded so the proposed safety-mobility
    # pathway can be examined in TEMPORAL order rather than inferred from
    # outcome differences alone: activation -> slowing -> stopping ->
    # deadlock or averted contact.
    ttc_active_frac: float = 0.0
    mean_speed_active: float = float('nan')   # speed while filter active
    mean_brake_active: float = float('nan')   # commanded brake factor while active
    recovery_episodes: int = 0
    recovery_active_steps: int = 0
    recovery_release_integral: float = 0.0
    recovery_exhausted: bool = False
    recovery_ended_progress: int = 0
    recovery_ended_duration: int = 0
    recovery_suppressed: int = 0
    mandatory_override_steps: int = 0
    route_advancements: int = 0
    route_replans: int = 0
    route_replans_failed: int = 0
    ttc_activations: int = 0
    ttc_brake_time: float = 0.0
    first_activation_step: int = -1     # -1 = never activated
    sustained_activation_step: int = -1  # first activation held >= 1.0 s
    deadlock_onset_step: int = -1        # first step of the stall window
    stop_episodes: int = 0
    stop_time: float = 0.0
    mean_speed: float = 0.0
    min_speed: float = 0.0
    time_to_goal: float = float("nan")
    unsafe_event_step: int = -1  # first step with any swept gap <= 0
    outcome: str = "timeout"
    steps: int = 0
    path_length: float = 0.0
    final_dist: float = 0.0
    final_speed: float = 0.0
    ped_collisions: int = 0
    obs_collisions: int = 0
    min_ped_gap: float = math.inf
    min_obs_gap: float = math.inf
    traj: np.ndarray = field(default_factory=lambda: np.zeros((2, 0)))


def run_trial(S: Scenario, planner: str = "sfm", log: bool = False,
              ttc: bool = False, a_ped_robot: float | None = None,
              ped_velocity_fn=None, recovery_cfg=None, trace=None,
              route_recovery=None) -> Result:
    from copy import replace as _replace
    P = S.params
    if ttc or a_ped_robot is not None:
        P = _replace(P, ttcEnabled=bool(ttc),
                     aPedRobot=a_ped_robot if a_ped_robot is not None else P.aPedRobot)
    nPed = S.ped_pos.shape[1]
    N = 1 + nPed

    x = np.zeros((2, N))
    v = np.zeros((2, N))
    x[:, 0] = S.robot_start
    v[:, 0] = S.robot_vel
    if nPed:
        x[:, 1:] = S.ped_pos
        v[:, 1:] = S.ped_vel

    radii = np.array([P.rRobot] + [P.rPed] * nPed)
    v0 = np.array([P.v0Robot] + [P.v0Ped] * nPed)

    goals = np.zeros((2, N))
    if nPed:
        goals[:, 1:] = S.ped_goal

    if planner == "hybrid" and S.waypoints is not None and S.waypoints.shape[1]:
        wi = 0
        goals[:, 0] = S.waypoints[:, 0]
    else:
        planner = "sfm"
        wi = 0
        goals[:, 0] = P.goal

    pool_idx = 0
    prev_brake = 1.0
    dwell = stall = 0
    act_steps = 0; activations = 0; was_active = False
    first_act = -1; sustained_act = -1; act_run = 0
    stop_steps = 0; stop_eps = 0; was_stopped = False
    speed_sum = 0.0; speed_min = math.inf
    act_speed_sum = 0.0; brake_sum = 0.0
    rec_active_steps = 0; mandatory_steps = 0
    _hold = getattr(S, 'blocker_hold', None) or {}
    _route = route_recovery
    _wps = S.waypoints if S.waypoints is not None else None
    _mc = getattr(S, "map_change", None)
    _obs = list(S.obstacles)
    _mc_done = False
    _pp = None; _rec = None
    if recovery_cfg is not None:
        if S.waypoints is None or S.waypoints.shape[1] < 2:
            raise ValueError("recovery requires a waypoint polyline")
        from path_progress import PathProgress
        from recovery import ProgressAwareRelease
        _pp = PathProgress(S.waypoints)
        _rec = ProgressAwareRelease(P, recovery_cfg)
    deadlock_onset = -1
    unsafe_step = -1
    path_length = 0.0
    ped_col = obs_col = 0
    in_ped = in_obs = False
    min_pg = min_og = math.inf
    traj = np.zeros((2, P.maxSteps)) if log else None

    R = Result()
    steps_taken = P.maxSteps

    for step in range(1, P.maxSteps + 1):
        # G2 map change: frozen time, arm-independent, seen by every arm.
        if _mc is not None and not _mc_done and step * P.dt >= _mc["t"]:
            _obs = _obs + list(_mc["obstacles"])
            _mc_done = True
            if _route is not None:
                _route.obstacles = _obs
                if getattr(_route, "planner", None) is not None:
                    _route.planner = _route.planner.__class__(
                        P.xMax, P.yMax, _obs, P.graphMargin)
        F, mpg, mog = compute_forces(x, v, goals, radii, v0, _obs, P)

        # semi-implicit Euler: velocity first
        v = v + F * P.dt
        sp = np.hypot(v[0], v[1])
        over = sp > P.vMax
        if over.any():
            v[:, over] = v[:, over] / sp[over] * P.vMax
        # no lower clamp

        # Optional external pedestrian driver (Amendment E). When absent the
        # loop is exactly as validated. When supplied it REPLACES pedestrian
        # velocities only -- the robot's column is untouched, so the
        # intervention under test is unchanged and the pedestrian model is the
        # only thing that varies. Applied after the speed cap and before the
        # TTC filter, which acts on the robot alone.
        if ped_velocity_fn is not None and nPed:
            v[:, 1:] = ped_velocity_fn(step, x, v, radii, goals)

        # TTC filter applied AFTER the force update and speed cap, so it is
        # the last authority on commanded speed and ablates cleanly.
        # --- Amendment G: time-based blocker hold -------------------------
        # Zeroes the DESIGNATED pedestrian's velocity only. No global pause,
        # no force-side effect, no change to any other agent. Release is
        # time-triggered from the scenario and never conditioned on robot
        # state. Durations are SECONDS, converted with the actual dt, so the
        # behaviour is identical at any timestep.
        if _hold:
            t_now = step * P.dt
            for _i, _h in _hold.items():
                if t_now < _h:
                    v[:, _i + 1] = 0.0

        cur_active, cur_b = False, 1.0
        if P.ttcEnabled and nPed:
            from ttc import ttc_brake_ex
            # EXECUTION ORDER, fixed and tested:
            #   state -> progress -> TTC command + flags -> recovery -> applied
            #   -> velocity -> position -> instrumentation
            # Recovery receives the TTC command and its emergency flags computed
            # from the SAME pre-command state. It never recomputes geometry.
            s_rem = _pp.remaining(x[:, 0]) if _pp is not None else float("nan")
            res = ttc_brake_ex(x[:, 0], v[:, 0], x[:, 1:], v[:, 1:],
                               radii[1:], P, prev_brake)
            b_cmd = res.scale
            mandatory = bool(res.hard_gap_active or res.urgent_ttc_active)
            if _rec is not None:
                b = _rec.update(b_cmd, bool(res.active), s_rem, mandatory)
            else:
                b = b_cmd
            v[:, 0] = v[:, 0] * b
            # The filter's slew state tracks its OWN command, not the applied
            # value, so the TTC layer is not driven by the recovery layer.
            # With recovery absent the two are identical, which is what makes
            # the disabled path bit-identical.
            prev_brake = b_cmd
            b_active = res.active
            cur_active, cur_b = bool(b_active), float(b)
            if _rec is not None:
                if _rec.state == "RECOVERY":
                    rec_active_steps += 1
                if mandatory:
                    mandatory_steps += 1
                if trace is not None:
                    trace.append(dict(step=step, ttc_scale=float(b_cmd),
                                      applied_scale=float(b),
                                      ttc_active=bool(b_active),
                                      mandatory=mandatory,
                                      recovery_active=_rec.state == "RECOVERY",
                                      episodes=_rec.episodes,
                                      release=float(b - b_cmd),
                                      s_remaining=float(s_rem),
                                      rearmed=_rec.rearmed))
            if b_active:
                act_steps += 1
                act_run += 1
                if not was_active:
                    activations += 1
                    if first_act < 0:
                        first_act = step
                # "Sustained" = held for a full second, the timescale at which
                # a brake becomes a stop rather than a momentary slow.
                if act_run >= int(1.0 / P.dt) and sustained_act < 0:
                    sustained_act = step - act_run + 1
            else:
                act_run = 0
            was_active = b_active

        x_prev = x.copy()
        x = x + v * P.dt

        if log:
            traj[:, step - 1] = x[:, 0]

        path_length += float(np.hypot(*(x[:, 0] - x_prev[:, 0])))
        rob_speed = float(np.hypot(*v[:, 0]))
        speed_sum += rob_speed
        speed_min = min(speed_min, rob_speed)
        # Tier-1 dose-response instrumentation (Amendment A.5 prediction 3):
        # speed and commanded brake factor CONDITIONAL on the filter being
        # active. Accumulators only -- they read state after it is committed
        # and never feed back into v, x or control flow.
        if cur_active:
            act_speed_sum += rob_speed
            brake_sum += cur_b
        stopped = rob_speed < P.stallSpeed
        if stopped:
            stop_steps += 1
            if not was_stopped:
                stop_eps += 1
        was_stopped = stopped

        gp, go = float(mpg[0]), float(mog[0])
        min_pg = min(min_pg, gp)
        min_og = min(min_og, go)
        if gp < 0:
            if unsafe_step < 0:
                unsafe_step = step
            if not in_ped:
                ped_col += 1
                in_ped = True
        else:
            in_ped = False
        if go < 0:
            if not in_obs:
                obs_col += 1
                in_obs = True
        else:
            in_obs = False

        if planner == "hybrid":
            # Amendment G: route recovery acts on the SAME waypoint array the
            # ordinary progression uses. Arms B and C differ only by whether
            # the object may replan; arm A passes route_recovery=None and this
            # block is byte-identical to the pre-Amendment engine.
            if _route is not None:
                new_wp, new_wi = _route.update(step, x[:, 0], wi)
                if new_wp is not None:
                    _wps = new_wp
                if new_wi is not None:
                    wi = int(new_wi)
            if (np.hypot(*(x[:, 0] - _wps[:, wi])) < P.rWaypoint
                    and wi < _wps.shape[1] - 1):
                wi += 1
            goals[:, 0] = _wps[:, wi]

        # pedestrian goals consumed in order from the pool
        for i in range(1, N):
            if np.hypot(*(x[:, i] - goals[:, i])) < P.pedGoalTol:
                if pool_idx >= S.goal_pool.shape[1]:
                    raise RuntimeError(f"goal pool exhausted at step {step}")
                goals[:, i] = S.goal_pool[:, pool_idx]
                pool_idx += 1

        dist_goal = float(np.hypot(*(x[:, 0] - np.asarray(P.goal))))
        if dist_goal <= P.rGoal and rob_speed <= P.vGoal and gp >= 0:
            dwell += 1
            if dwell >= P.dwellSteps:
                R.outcome = "success"
                steps_taken = step
                break
        else:
            dwell = 0

        if rob_speed < P.stallSpeed and dist_goal > P.rGoal:
            stall += 1
            if stall >= P.stallSteps:
                R.outcome = "deadlock"
                steps_taken = step
                # Onset is where the stall BEGAN, not where it was confirmed.
                # Temporal ordering against activation is meaningless otherwise.
                deadlock_onset = step - P.stallSteps + 1
                break
        else:
            stall = 0

    R.steps = steps_taken
    R.ttc_active_frac = act_steps / max(steps_taken, 1)
    if _route is not None:
        R.route_advancements = _route.n_adv
        R.route_replans = _route.n_plan
        R.route_replans_failed = _route.n_plan_failed
    R.recovery_active_steps = rec_active_steps
    R.mandatory_override_steps = mandatory_steps
    if _rec is not None:
        R.recovery_episodes = _rec.episodes
        R.recovery_exhausted = _rec.exhausted
        R.recovery_release_integral = _rec.release_integral
        R.recovery_ended_progress = _rec.ended_progress
        R.recovery_ended_duration = _rec.ended_duration
        R.recovery_suppressed = _rec.suppressed
    R.mean_speed_active = act_speed_sum / act_steps if act_steps else float('nan')
    R.mean_brake_active = brake_sum / act_steps if act_steps else float('nan')
    R.ttc_activations = activations
    R.ttc_brake_time = act_steps * P.dt
    R.first_activation_step = first_act
    R.sustained_activation_step = sustained_act
    R.deadlock_onset_step = deadlock_onset
    R.stop_episodes = stop_eps
    R.stop_time = stop_steps * P.dt
    R.mean_speed = speed_sum / max(steps_taken, 1)
    R.min_speed = 0.0 if speed_min == math.inf else speed_min
    R.time_to_goal = steps_taken * P.dt if R.outcome == "success" else float("nan")
    R.unsafe_event_step = unsafe_step
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
