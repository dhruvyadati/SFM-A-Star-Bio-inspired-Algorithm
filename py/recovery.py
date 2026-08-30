"""Progress-aware TTC release: a recovery layer for the cross-model failure.

DRAFT — not frozen until Amendment F is written and the ORCA-responsive study
is archived.

TARGET. The one failure that is robust across every model tested is

    sustained TTC intervention -> loss of progress -> deadlock

It holds under SFM pedestrians (deadlock 2.0% -> 14.0%) and under both
ORCA-responsive responsibility modes (2.0% -> 21.6%, 2.2% -> 25.0%). The
CONTACT story is not robust -- it reverses with a 0.20 m radius change -- so
this layer deliberately does NOT target contacts.

PRINCIPLE. The filter is not wrong to brake; it is wrong to keep braking when
braking has stopped buying anything. Recovery triggers on sustained
intervention CONJOINED WITH insufficient progress, never on either alone:

  * intervention alone is often correct and short-lived;
  * slow progress alone happens in dense crowds without any filter involvement.

SAFETY FLOOR THAT IS NEVER RELAXED. Recovery scales back the discretionary
brake only. Below `ttcHardGap` the emergency stop still applies at full
authority, so the layer cannot trade contact for mobility at close range --
which is exactly the trade the confirmatory study showed is easy to make by
accident.

STATE MACHINE

    NORMAL  --(active_frac_window >= THETA_ACT and progress_window < THETA_PROG)-->  RECOVERY
    RECOVERY --(progress_window >= THETA_PROG held for EXIT_HOLD, or elapsed > MAX_RECOVERY)--> NORMAL

Hysteresis on exit prevents chatter at the trigger boundary; MAX_RECOVERY
bounds how long the discretionary brake can be suppressed even if progress
never resumes, so a genuinely blocked robot returns to full caution.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class RecoveryConfig:
    window: float = 3.0          # s, trailing window for both triggers
    theta_act: float = 0.80      # fraction of window with the filter active
    theta_prog: float = 0.20     # m of goal-ward progress required over window
    relax: float = 0.60          # how far the discretionary brake is released
    exit_hold: float = 1.0       # s of restored progress before leaving RECOVERY
    max_recovery: float = 5.0    # s cap on any single recovery episode
    max_episodes: int = 2        # per trial; after this, full TTC is restored
    cooldown: float = 2.0        # s of RESTORED PROGRESS before re-arming
    enabled: bool = True


class ProgressAwareRelease:
    """Wraps the TTC brake factor with a progress-aware release.

    Call `update` once per step with the brake factor the filter commanded, the
    robot's distance to its current target, and whether the hard-gap emergency
    condition is active. Returns the brake factor to apply.
    """

    def __init__(self, P, cfg: RecoveryConfig):
        self.P, self.cfg = P, cfg
        # N SAMPLES SPAN (N-1) INTERVALS. int(window/dt) gives 300 samples
        # for a 3.0 s window, spanning only 2.99 s -- a one-step boundary drift
        # of exactly the kind the protocol exists to eliminate.
        if abs(cfg.window / P.dt - round(cfg.window / P.dt)) > 1e-9:
            raise ValueError(f"window {cfg.window} is not a multiple of dt {P.dt}")
        n = int(round(cfg.window / P.dt)) + 1
        self.active_hist = deque(maxlen=n)
        self.dist_hist = deque(maxlen=n)
        self.state = "NORMAL"
        self.recovery_steps = 0
        self.exit_steps = 0
        self.episodes = 0
        self.recovery_step_count = 0
        self.rearm_steps = 0         # consecutive steps of restored progress
        self.rearmed = True          # LATCH: first episode needs no re-arm
        self.exhausted = False       # episode cap reached; full TTC restored
        # Episode counts alone cannot explain behaviour, so every termination
        # is attributed and every suppressed trigger is counted.
        self.ended_progress = 0      # episodes ended by restored progress
        self.ended_duration = 0      # episodes ended by the duration cap
        self.suppressed = 0          # triggers blocked after exhaustion
        self.release_integral = 0.0  # sum of (applied - commanded) over steps
        self.last_end_reason = None

    def update(self, brake: float, active: bool, s_remaining: float,
               hard_gap_active: bool) -> float:
        """`s_remaining` is remaining ARC LENGTH on the fixed A* polyline.

        Straight-line distance to the current target was used in the first
        draft and is wrong: rounding an obstacle produces genuine arc-length
        progress while that distance stagnates, so the detector would fire in
        exactly the cluttered geometry it must handle carefully.
        """
        cfg, dt = self.cfg, self.P.dt
        self.active_hist.append(1.0 if active else 0.0)
        self.dist_hist.append(float(s_remaining))

        if not cfg.enabled or len(self.dist_hist) < self.dist_hist.maxlen:
            return brake

        act_frac = float(np.mean(self.active_hist))
        # Positive = arc length consumed. Negative when pushed backwards along
        # the route, which is correct and is NOT clamped.
        progress = self.dist_hist[0] - self.dist_hist[-1]

        # RE-ARM ON DEMONSTRATED PROGRESS, NOT ELAPSED TIME. A timer alone
        # lets a permanently stalled robot wait out the interval and open
        # another episode with no evidence the situation changed, which would
        # partition one continuous stall into repeated artificial episodes and
        # defeat the cap. The counter resets the moment progress lapses.
        if self.state == "NORMAL" and not self.rearmed:
            if progress >= cfg.theta_prog:
                self.rearm_steps += 1
                # LATCHED once earned. A live counter cannot work: the trigger
                # itself requires progress BELOW threshold, so a counter that
                # resets on every lapse is always 0 at the moment of
                # triggering and no second episode could ever start.
                if self.rearm_steps * dt >= cfg.cooldown:
                    self.rearmed = True
            else:
                self.rearm_steps = 0

        if self.state == "NORMAL":
            # An episode may start only when the trigger holds AND the cap is
            # not spent AND the cooldown has elapsed. The cooldown prevents one
            # continuous stall from being re-entered repeatedly as a sequence
            # of short episodes, which would defeat the cap entirely.
            triggering = (act_frac >= cfg.theta_act and progress < cfg.theta_prog)
            # BUDGET CHECKED AT TRIGGER TIME, not only after an episode ends.
            # Evaluating exhaustion solely on exit enforces the cap one episode
            # late: it is correct for max_episodes >= 1 but lets a cap of 0
            # start one episode, and in general asks "have I overspent?" rather
            # than "may I spend?".
            budget_left = self.episodes < cfg.max_episodes
            if not budget_left:
                self.exhausted = True
            if triggering and (not budget_left or not self.rearmed):
                self.suppressed += 1
            if budget_left and self.rearmed and triggering:
                self.state = "RECOVERY"
                self.rearmed = False
                self.rearm_steps = 0
                self.recovery_steps = 0
                self.exit_steps = 0
                self.episodes += 1
        else:
            self.recovery_steps += 1
            self.recovery_step_count += 1
            if progress >= cfg.theta_prog:
                self.exit_steps += 1
                if self.exit_steps * dt >= cfg.exit_hold:
                    self.state = "NORMAL"
                    self.last_end_reason = "restored_progress"
                    self.ended_progress += 1
            else:
                self.exit_steps = 0
            if self.state == "RECOVERY" and self.recovery_steps * dt >= cfg.max_recovery:
                self.state = "NORMAL"
                self.last_end_reason = "duration_cap"
                self.ended_duration += 1
            if self.state == "NORMAL":
                # Episode just ended, by restored progress or by the duration
                # cap. Start the cooldown and retire the controller once the
                # episode budget is spent -- after which the ordinary 5 s stall
                # rule classifies the trial, with full TTC authority restored.
                if self.episodes >= cfg.max_episodes:
                    self.exhausted = True   # also set at trigger time

        if self.state != "RECOVERY":
            return brake
        # The emergency floor is absolute: recovery never touches it.
        if hard_gap_active:
            return brake
        # Release the discretionary brake toward free motion, never beyond it.
        applied = float(brake + (1.0 - brake) * cfg.relax)
        self.release_integral += (applied - brake) * dt
        return applied
