# Amendment H (DRAFT) — bounded external-benchmark transfer check

**Answers reviewer 1.40.** Frozen before any benchmark is installed, so the
feasibility criterion cannot be relaxed once effort has been sunk into an
integration.

## The question, and what it is not

> Does the observed TTC safety–mobility trade-off appear when the controller is
> exercised **outside** the custom MATLAB/Python simulator?

**This is a transfer check, not a second confirmation.** It does not need to
reproduce Δ = +0.078. It tests whether the DIRECTION transfers: fewer unsafe
outcomes with TTC, more immobility or slower completion with TTC.

**The Python port does not answer 1.40 and must never be presented as doing
so.** It re-implements the same equations on the same scenarios; agreement to
1e-14 establishes implementation reproducibility, not external validity. The
ORCA-responsive study is closer but still varies pedestrian dynamics *inside*
our own framework — model generalisation, not external validation.

## Design — deliberately small

| | |
|---|---|
| Arms | guided SFM vs guided SFM–TTC. **Two.** |
| TTC parameters | the frozen defaults. No tuning in the external environment. |
| Recovery (Amendment F) | **not included** |
| Route recovery (Amendment G) | **not included** |
| Scenarios | a few prespecified representative ones, named before running |
| Repetition | repeated seeds / pedestrian initialisations where the environment supports it |

Outcomes, and only these: safe completion; unsafe/contact; deadlock or failure
to progress; goal completion; minimum clearance; traversal time.

## What must stay separate

Combining any of these would make attribution impossible:

1. **H** — transfer of the guided ± TTC pattern
2. **F** — progress-aware relaxation of TTC braking
3. **G** — waypoint advancement and replanning
4. baseline study — ORCA / DWA / A\*-replanning comparisons
5. scenario diversity — bottlenecks, crossing flows, groups, sudden obstruction

## STOP RULE — frozen before installation

> Continue with the selected external benchmark **only if** the existing guided
> controller can be integrated, one scenario can be executed reproducibly, and
> the required outcome fields can be extracted **without altering the
> controller architecture**.

Four conditions, each pass/fail, evaluated during a time-boxed spike:

| # | Condition |
|---|---|
| H1 | the environment installs in this environment |
| H2 | the controller can receive state and issue velocity commands |
| H3 | the six outcome fields can be logged consistently |
| H4 | one canonical scenario runs deterministically on repeat |

**If any fails, document the attempt and take Path B** — a sharply narrowed
simulation-only claim. Reviewer 1.40 must not become an open-ended robotics
infrastructure project, and the failure itself is reportable evidence about
benchmark availability.

**Altering the controller to fit the benchmark fails the spike.** If the
architecture has to change, the thing being tested is no longer the thing that
was confirmed.

## SPIKE RESULT — 4 August 2026, CrowdNav: ALL FOUR CONDITIONS PASS

| # | Condition | Result |
|---|---|---|
| H1 | environment installs | **PASS** — cloned from GitHub (not on PyPI); needs `gym`, `matplotlib`, `scipy`, `gitpython`, and `rvo2` (already present for Amendment E) |
| H2 | controller receives state, issues commands | **PASS** — 18 state observations, velocity commands accepted |
| H3 | outcome fields extractable | **PASS** — `info` classes map directly: `ReachGoal`→goal, `Collision`→unsafe, `Timeout`→deadlock/no-progress, `Danger`→clearance breach |
| H4 | one scenario replays deterministically | **PASS** — identical trajectories and outcome across repeat runs |

**Why CrowdNav suits the stop rule.** Its `Policy` base class exposes an
abstract `predict(state)` returning `ActionXY(vx, vy)` — a velocity command,
which is exactly what SFM+TTC already produces. Our controller wraps as a
policy **without any architectural change**, which is the condition that would
otherwise have failed the spike.

**Two frictions, both recorded, neither disqualifying:**

1. CrowdNav targets a pre-0.21 gym API. Modern gym's `PassiveEnvChecker`
   requires an `action_space` the environment never defines, so `gym.make`
   fails. Instantiating `CrowdSim()` directly bypasses it. **This adapts the
   benchmark harness, not our controller**, so the stop rule holds.
2. Installed `gym` warns it is unmaintained and unsupported on NumPy 2.x. It
   worked here; if it breaks, `gymnasium` is the drop-in successor. Recorded so
   a later failure is not mistaken for a result.

The spike used a deliberately trivial straight-to-goal policy, which collided —
as expected, since it ignores pedestrians. That is not a navigation result; it
demonstrates the interface works and that `Collision` is detectable.

**Decision: proceed with CrowdNav.** SocNavBench and ROS/Gazebo are not
attempted; the stop rule says stop at the first candidate passing all four.

## FROZEN DESIGN — 4 August 2026, before the adapter exists

> The external transfer study uses two native CrowdNav scenario families,
> circle crossing and square crossing, each at 5 and 10 pedestrians, with 50
> paired seeds per stratum. The guided SFM and guided SFM–TTC controllers are
> compared without parameter tuning in CrowdNav. TTC parameters remain fixed at
> the confirmatory setting. The study evaluates whether the previously observed
> reduction in unsafe outcomes and increase in immobility transfer to an
> independently implemented crowd-navigation environment.

| | |
|---|---|
| Strata | `circle_crossing` x {5, 10} humans; `square_crossing` x {5, 10} humans = **4** |
| Seeds per stratum | **50** paired |
| Episodes per arm | 200 |
| Arms | guided SFM; guided SFM–TTC. **Two.** |
| **Total runs** | **400** |

No static-obstacle or narrow-passage variants: CrowdNav is a crowd-interaction
benchmark, and blockage questions belong to Amendment G and reviewer 1.36.

### Primary question — directional and joint

> Does TTC reduce collision/unsafe outcomes **while** increasing timeout or
> loss-of-progress outcomes, relative to the same guided SFM controller in
> CrowdNav?

**Not** whether +0.078 reproduces. Ordered interpretation:

| Observation | Reading |
|---|---|
| collision down, deadlock/timeout up | trade-off transfers |
| collision down, no mobility cost | external environment more favourable |
| collision unchanged, mobility worse | only the cost transfers |
| neither changes | no transfer detected |
| collision up | adverse transfer |

### Outcomes — CrowdNav-native, plus our own deadlock measure

`ReachGoal` → goal completion · `Collision` → unsafe collision ·
`Timeout` → timeout/failure to complete · `Danger` min distance → clearance.

**A CrowdNav timeout is NOT our deadlock.** Timeout means the episode horizon
ended; it does not establish that the robot met the frozen 5 s low-speed
criterion. The adapter therefore instruments our own measure —
*speed < 0.05 m/s continuously for 5.0 s while outside the goal radius* — and
**both are reported separately**.

### Pairing

On **initial state and random stream**, not on identical realised pedestrian
trajectories. CrowdNav humans react to the robot, so trajectories legitimately
diverge between arms once the robot behaves differently.

### Environment provenance — recorded, not assumed

    CrowdNav commit   20d678085c06831e658a65b9e20c8bb6f6ecdc10
    CrowdNav remote   https://github.com/vita-epfl/CrowdNav.git  (2021-10-02)
    instantiation     CrowdSim() directly; gym.make bypassed
    gym 0.26.2 | numpy 2.3.1 | scipy 1.16.3 | matplotlib 3.10.8 | rvo2 (no __version__)
    time_step 0.25 s | circle_radius 4 | square_width 10
    humans: policy=orca, radius 0.3, v_pref 1.0, visible=true
    robot:  radius 0.3, v_pref 1.0, visible=FALSE

## D1 and D2 — DECIDED 4 August 2026, before the adapter was written

### D1 — native 0.25 s as primary; no sub-stepping

> The primary external-transfer analysis uses CrowdNav's native 0.25 s
> simulation and control interval. Controller parameters defined in seconds are
> evaluated using the actual elapsed interval, including the TTC slew limit. No
> internal robot-only substepping is used because holding pedestrian
> observations fixed while evolving the robot would introduce asynchronous
> hybrid dynamics not native to either simulator. A prespecified sensitivity
> analysis repeats 10 paired seeds per stratum at 0.01 s to assess timestep
> dependence; it cannot alter the primary result or controller parameters.

**Correction to an earlier statement in this document.** It described
`ttcSlewRate * dt = 1.0` at dt = 0.25 as leaving the rate limiter with "no
effective limiting". That was wrong. The limiter is not mathematically
disabled: the configured *continuous-time* rate simply permits the full scale
range to be traversed within one native control interval. What is genuinely
lost is the **within-step brake ramp**, because CrowdNav accepts one constant
velocity action per interval.

Sub-stepping was rejected, not merely deprioritised: evolving the robot at
0.01 s while pedestrian observations stay frozen for 0.25 s reproduces neither
simulator, and re-evaluating an unchanged pedestrian threat 25 times would
manufacture false TTC persistence.

The study therefore transfers the **controller architecture and its physical
time constants**, not a bit-identical replay of the 100 Hz implementation.

### D2 — invisible primary, visible as a full prespecified sensitivity

> The primary transfer analysis uses CrowdNav's native robot-invisible
> configuration, in which human ORCA policies do not react to the robot. A
> prespecified sensitivity analysis repeats the complete design with the robot
> visible to human policies. This second analysis changes pedestrian-robot
> interaction and is interpreted as a model-responsiveness sensitivity, not as
> a replication or replacement of the primary analysis.

Running only `false` would risk reproducing the original effect by preserving
effectively non-responsive pedestrians; running only `true` could erase the
rescue pool through the ceiling mechanism already seen in Amendment E. The two
answer different questions and both are run.

## FINAL RUN MATRIX — 960 runs

| Study | Strata | Seeds | Arms | visible | dt | Runs |
|---|---|---|---|---|---|---|
| **Primary transfer** | 4 | 50 | 2 | false | 0.25 | **400** |
| Visibility sensitivity | 4 | 50 | 2 | true | 0.25 | 400 |
| Timestep sensitivity | 4 | **first 10, prespecified** | 2 | both | 0.01 | 160 |

The timestep-sensitivity subset is **seeds 1-10 in each stratum, fixed now** —
not chosen after viewing primary outcomes.

## SCOPE LIMIT — CrowdNav cannot test the guidance factor

CrowdNav's core environment has **no static obstacles**. A\* therefore returns a
straight line to the goal, and global guidance is trivially satisfied in both
arms.

**Consequence, which must appear in the manuscript.** The paper's headline is an
*interaction*: global guidance determines whether TTC filtering helps. This
study can only transfer the **TTC main effect under (degenerate) guidance**. It
cannot test the interaction, because there is no unguided arm to contrast and
nothing for guidance to plan around.

That is not a reason to skip it — the TTC safety-mobility trade-off is the
contested half — but the transfer claim must be worded as:

> the TTC trade-off was examined in an independent crowd-navigation
> environment; the guidance x TTC interaction was not, because that benchmark
> contains no static obstacles for global planning to resolve.

Obstacle-dependent questions remain with Amendment G and reviewer 1.36.

## Implementation requirements, fixed before coding

1. **Durations from elapsed time, never hard-coded step counts.** The 5.0 s
   deadlock criterion is 20 intervals at dt = 0.25 and 500 at dt = 0.01.
   Compute from elapsed time and assert exact divisibility; a hard-coded count
   would be silently wrong in one of the two conditions.
2. **Log the commanded TTC scale at every step, and whether the requested
   change was actually clipped by the slew bound.** This measures whether the
   limiter is inactive in practice at dt = 0.25, rather than inferring it from
   the maximum possible change — which is what produced the incorrect claim
   corrected above.
3. Report CrowdNav `Timeout` and our own 5 s low-speed deadlock **separately**.

## Candidates, in order of expected tractability

| Benchmark | Expected difficulty | Risk |
|---|---|---|
| **CrowdNav** | Python, gym-style, controller plugs in as a policy | actively used; interface must be written |
| **SocNavBench** | Python, but unmaintained; large data download | maintenance status was flagged as a concern early |
| **ROS/Gazebo** | heaviest: full stack, message plumbing | highest cost, least suited to a bounded spike |

Try in that order. Stop at the first that passes all four conditions.

## Reporting, whatever the outcome

- Transfer observed → the trade-off is not an artefact of the custom simulator.
- Transfer not observed → equally reportable, and a genuine limit on external
  validity. It must not be suppressed or re-framed.
- Spike fails → Path B, with the attempt and the failure documented in the
  response letter rather than omitted.

Sample size, scenario names and the exact per-arm comparison are fixed **after**
the spike passes and before any outcome is inspected.
