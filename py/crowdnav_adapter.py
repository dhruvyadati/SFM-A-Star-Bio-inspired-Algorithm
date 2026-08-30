"""Amendment H: direct-goal SFM(+TTC) controller as a CrowdNav policy.

ARM NAMING. These are "direct-goal SFM" and "direct-goal SFM-TTC", NOT "guided"
-- CrowdNav's native scenarios contain no static obstacle-routing problem, so
the A* route degenerates to a straight start-to-goal reference. Calling them
"guided" would imply the global planner is doing work it is not doing here.

The controller architecture is UNCHANGED. This wraps it in CrowdNav's own
`Policy` interface — `predict(JointState) -> ActionXY(vx, vy)` — which happens
to be exactly the shape our controller already produces. If this file had
needed to alter the control law, the feasibility stop rule would have failed.

WHAT IS AND IS NOT TRANSFERRED. The controller's ARCHITECTURE and its PHYSICAL
TIME CONSTANTS transfer; the 100 Hz implementation does not. CrowdNav accepts
one constant velocity per 0.25 s interval, so the within-step brake ramp is
absent by construction. No robot-only substepping is performed: evolving the
robot at 0.01 s while pedestrian observations stay frozen for 0.25 s would
reproduce neither simulator, and would manufacture false TTC persistence by
re-evaluating an unchanged threat 25 times.

CrowdNav has NO STATIC OBSTACLES, so the A* layer is degenerate — a straight
line to the goal. This study transfers the TTC main effect only; the guidance x
TTC interaction is untestable here and is not claimed.

Durations are expressed in SECONDS and converted using the environment's actual
timestep, with exact divisibility asserted. A hard-coded step count would be
silently wrong at one of the two timesteps under study (20 intervals at 0.25 s,
500 at 0.01 s).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from sfm_explore import Params, compute_forces
from ttc import ttc_brake_ex

CROWDNAV = Path(__file__).resolve().parents[2] / "external" / "CrowdNav"
if str(CROWDNAV) not in sys.path:
    sys.path.insert(0, str(CROWDNAV))

from crowd_sim.envs.policy.policy import Policy          # noqa: E402
from crowd_sim.envs.utils.action import ActionXY         # noqa: E402

DEADLOCK_SECONDS = 5.0      # frozen: the confirmatory low-speed criterion
STALL_SPEED = 0.05          # m/s, frozen


class SfmTtcPolicy(Policy):
    """Guided SFM, optionally with the TTC safety filter.

    `ttc=False` is the reference arm; `ttc=True` uses the confirmatory default
    parameters with NO tuning in CrowdNav.
    """

    def __init__(self, ttc: bool, dt: float, params: Params | None = None):
        super().__init__()
        self.trainable = False
        self.multiagent_training = True
        self.kinematics = "holonomic"

        P = params or Params()
        # The controller's seconds-denominated constants are unchanged; only
        # the integration interval differs, and it comes from the environment.
        self.P = type(P)(**{**P.__dict__, "dt": float(dt), "ttcEnabled": bool(ttc)})
        self.use_ttc = bool(ttc)
        self.dt = float(dt)

        n = DEADLOCK_SECONDS / self.dt
        assert abs(n - round(n)) < 1e-9, (
            f"deadlock window {DEADLOCK_SECONDS}s is not a whole number of "
            f"{self.dt}s steps")
        self.deadlock_steps = int(round(n))

        self.reset_episode()

    def reset_episode(self):
        self.prev_scale = 1.0
        self.stall_steps = 0
        self.deadlock = False
        self.deadlock_step = -1
        self.step_i = 0
        self.n_clipped = 0
        self.max_requested = 0.0
        self.trace = []

    def configure(self, config):
        return

    def predict(self, state):
        """JointState -> ActionXY. One evaluation per environment step."""
        s = state.self_state
        humans = state.human_states
        n = len(humans)

        # --- assemble the same arrays the confirmed controller consumes ----
        N = 1 + n
        x = np.zeros((2, N)); v = np.zeros((2, N))
        goals = np.zeros((2, N)); radii = np.zeros(N); v0 = np.zeros(N)
        x[:, 0] = (s.px, s.py); v[:, 0] = (s.vx, s.vy)
        goals[:, 0] = (s.gx, s.gy); radii[0] = s.radius; v0[0] = s.v_pref
        for i, h in enumerate(humans, start=1):
            x[:, i] = (h.px, h.py); v[:, i] = (h.vx, h.vy)
            # Pedestrian goals are not observable in CrowdNav. They are driven
            # by the environment's own ORCA policies, so the SFM pedestrian
            # terms are irrelevant here: only the ROBOT's row is used. Setting a
            # human's goal to its own position leaves its force undefined-but-
            # unused, and never feeds back into the environment.
            goals[:, i] = x[:, i]
            radii[i] = h.radius; v0[i] = 0.0

        # A* is degenerate without obstacles: head for the goal directly.
        F, _, _ = compute_forces(x, v, goals, radii, v0, [], self.P)

        # D3: speed cap scaled to the ENVIRONMENT's speed scale.
        # CrowdNav does not clamp actions (check_validity only checks type), so
        # our vMax = 2.0 would let the robot outrun every human in a benchmark
        # whose agents use v_pref = 1.0 -- changing collision dynamics and
        # misusing the environment. The confirmed controller's ratio
        # vMax / v0Robot = 2.0 / 1.3 is preserved and applied to the
        # environment's own v_pref, so the controller keeps its shape while
        # respecting the benchmark's speed scale.
        v_max = (self.P.vMax / self.P.v0Robot) * float(s.v_pref)
        vel = v[:, 0] + F[:, 0] * self.dt
        sp_req = float(np.hypot(*vel))
        clipped = sp_req > v_max
        if clipped:
            vel = vel / sp_req * v_max
        self.n_clipped += int(clipped)
        self.max_requested = max(self.max_requested, sp_req)

        scale, raw, limited, active, mandatory = 1.0, 1.0, False, False, False
        if self.use_ttc and n:
            r = ttc_brake_ex(x[:, 0], vel, x[:, 1:], v[:, 1:],
                             radii[1:], self.P, self.prev_scale)
            scale, raw, limited = r.scale, r.raw_scale, r.slew_limited
            active = bool(r.active)
            mandatory = bool(r.hard_gap_active or r.urgent_ttc_active)
            vel = vel * scale
            self.prev_scale = scale

        # --- our own deadlock measure, independent of CrowdNav Timeout ------
        speed = float(np.hypot(*vel))
        d_goal = float(np.hypot(s.gx - s.px, s.gy - s.py))
        if speed < STALL_SPEED and d_goal > s.radius:
            self.stall_steps += 1
            if self.stall_steps >= self.deadlock_steps and not self.deadlock:
                self.deadlock = True
                self.deadlock_step = self.step_i
        else:
            self.stall_steps = 0

        self.trace.append(dict(step=self.step_i, requested_speed=sp_req,
                               applied_speed=speed, clipped=bool(clipped),
                               v_max=float(v_max), ttc_scale=float(scale),
                               raw_scale=float(raw), slew_limited=bool(limited),
                               ttc_active=active, mandatory=mandatory,
                               speed=speed, dist_goal=d_goal))
        self.step_i += 1
        return ActionXY(float(vel[0]), float(vel[1]))
