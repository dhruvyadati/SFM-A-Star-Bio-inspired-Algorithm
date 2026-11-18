# Confirmatory Protocol — FROZEN

**Written 31 July 2026, before any confirmatory outcome exists.**

This document is fixed. Anything changed after confirmatory data are produced
must be reported as a deviation, with the reason and the date.

| | |
|---|---|
| Seed set SHA-256 | `fc5905167372c689a637f4274ba11885ac0f0120a0ea77eb035952e46f511f1d` |
| Seed base | 31415926, disjoint from pilot (19730401), deadlock batch (20260729), development (77712345), fixtures (90210) |
| Manifest | `v2/runs/v2/confirm_manifest.json` |
| Code hashes | `sfm.py 5b87920431375303` · `ttc.py b89dcad9104b6db9` · `RunTrial.m 47ed2452e17933c2` · `ComputeForces.m 49165196ba68c682` · `SfmParams.m aa7d0f2329cedfa4` |

**Status of the design change.** The primary question was narrowed from the
guidance×filter interaction to the guided-filter simple effect. This was a
**prespecified development-stage decision rule** applied after the 300-scenario
development set and before any confirmatory seed was touched. It is *not*
preregistration — no dated protocol existed before development. The rule was:
run confirmation only if the affordable N gives ≥80% power for the smallest
meaningful effect; otherwise narrow the claim before confirmation.

---

## 1. Primary question

Does TTC filtering improve safe-goal success in the A\*-guided hybrid controller?

$$H_0:\ p_{\text{hybrid-TTC}} - p_{\text{hybrid}} = 0$$

**Two-sided**, α = 0.05. Two-sided because the filter could plausibly *reduce*
safe success — it demonstrably increases deadlock, and a one-sided test would
assume away the direction the development data warn about.

## 2. Primary endpoint

Binary **safe goal**, per scenario per arm. All four must hold:

1. outcome is `success` — within 0.5 m of the goal, speed ≤ 0.1 m/s, held 1.0 s, no contact during the dwell;
2. zero pedestrian contact episodes over the whole trial (true body radii, swept check);
3. zero obstacle contact episodes;
4. not `deadlock`, not `timeout`.

Contact is surface gap ≤ 0 using **true** radii (0.35 m robot, 0.30 m pedestrian).
Any control-side safety buffer is never used for scoring.

This definition is final and will not be revised after confirmation.

## 3. Sample size

| | |
|---|---|
| Paired scenarios | **500** |
| Arms per scenario | 4 (`sfm`, `sfm-ttc`, `hybrid`, `hybrid-ttc`) |
| Total runs | **2000** |
| Allocation | 125 per condition × {baseline, dense, cluttered, bigObs} |

**Smallest meaningful effect: 0.075** — an absolute 7.5-point increase in
safe-goal probability is the smallest improvement that would justify adding the
filter. Chosen on that ground, not to preserve significance, and deliberately
below the development estimate of 0.110 because that estimate has shrunk twice
already (interaction: 0.400 → 0.194 → 0.087).

Simulated power at N=500, using the development discordance rate 0.283
[95% CI 0.233, 0.337]: ≈0.80–0.85 at Δ=0.075, ≈0.98 at Δ=0.10.

## 4. Primary estimand

Equal-condition benchmark average:

$$\Delta = \frac{1}{4}\sum_{k=1}^{4}\left[p_k(\text{hybrid-TTC}) - p_k(\text{hybrid})\right]$$

A constructed benchmark over four prespecified condition families — **not** a
deployment-wide effect. Equal weights prevent any single condition dominating,
and the conditions are benchmark categories, not frequency estimates.

## 5. Primary analysis

- Exact paired McNemar (binomial) on discordant pairs, two-sided, α = 0.05.
- **Stratified by condition**; stratum effects combined with equal weights per §4.
- Confidence interval: percentile bootstrap over scenarios within strata, 10 000 resamples.
- **Ties** (concordant pairs) contribute no information and are excluded from the McNemar statistic but retained in the rate estimates.
- **Timeouts count as failures** on the primary endpoint.
- **Failed or crashed runs**: the whole scenario is dropped from all four arms and reported by count. No retries, no substitution.
- The power simulation and the final analysis use the identical statistic.

## 6. Secondary hierarchy — ordered, prespecified

1. Unsafe-arrival difference, hybrid-TTC vs hybrid
2. Deadlock difference, hybrid-TTC vs hybrid
3. Goal-arrival difference, hybrid-TTC vs hybrid
4. Guidance effect with TTC (sfm-ttc vs hybrid-ttc)
5. Guidance effect without TTC (sfm vs hybrid)
6. TTC effect under unguided SFM (sfm vs sfm-ttc)
7. **Guidance × TTC interaction** — retained as a secondary factorial result. The study is *not* primarily powered for it; report estimate and interval.
8. Condition-specific effects — descriptive, not powered
9. Contact burden, minimum clearance, traversal time, filter duty cycle

Reported with confidence intervals and emphasis on estimates. Unadjusted
p-values below the primary are **not** separate confirmatory discoveries.

## 7. Decision rules — fixed in advance

Interpretation rests on the estimate and interval, not on `p < 0.05`.

| Result | Conclusion |
|---|---|
| Interval entirely above 0.075 | Strong evidence of a practically meaningful safe-goal improvement under guidance |
| Positive, interval includes 0.075 | Evidence of some benefit, insufficient that it reaches the meaningful magnitude |
| Interval includes 0 | The development benefit did not independently replicate |
| Positive safe-goal effect with large deadlock increase | Improvement on the safety endpoint at a substantial mobility cost — reported as a trade-off, not an unqualified gain |

Failure to reject does **not** establish equivalence. The interval determines
whether meaningful effects remain plausible.

## 8. Execution constraints

- Confirmation runs **once**. No re-runs, no parameter changes, no silent retries.
- Development data are closed. No further development scenarios.
- Scenario generation, engine, filter and analysis code are hashed above; any change invalidates the run.

## 9. Standing interpretation notes

**Mechanism.** The development pattern — lower goal arrival, higher deadlock,
guidance-dependent net safety — is *consistent with* a safety–mobility trade-off
induced by the filter. Establishing the mechanism directly would require
mediating variables (filter-active time, speed reduction, stationary time, stop
events and durations, minimum TTC, deadlock onset relative to activation).
Those are recorded but the mechanism is not claimed as proven.

**Unguided filter effect.** Development gave +0.023 [−0.003, +0.053], p = 0.17.
This is "no clear benefit detected", not "no benefit". Demonstrating practical
absence would need an equivalence margin defined in advance.

**Validation scope.** The Python implementation is validated against the MATLAB
reference — both implement the same equations. This says nothing about whether
those equations describe real robots or pedestrians.

---

# Amendment A — Mechanism analyses

**Added 31 July 2026, before any confirmatory outcome existed.** The seed set
is unchanged; `py/sfm.py` gained mediator instrumentation only. Recorded here
as a deviation with its reason and date, per §8.

Updated hash: `py/sfm.py 5e7d40f0ea8e229a` (was `5b87920431375303`).
All other files unchanged. Full set in `v2/runs/v2/confirm_manifest.json`
under `code_amended_2026_07_31`.

*Correction, same day, still before any confirmatory outcome: this line
initially carried a placeholder hash written before the value was
computed. Recorded rather than silently overwritten.*

## A.1 Why the primary run cannot establish the mechanism

Confirmation can show that TTC raises safe-goal success under guidance, lowers
goal completion, raises deadlock, and does less without guidance. That is an
**outcome pattern**, not an internal pathway. The proposed pathway is:

> activation → reduced speed / stopping → deadlock *or* averted contact → net
> safe-goal outcome

Establishing it needs the intermediate steps observed in **temporal order**.

## A.2 Prespecified mediator analyses — secondary, exploratory

Run only after the frozen primary analysis. Reported with intervals, not as
confirmatory discoveries.

**1. Activation and motion response.** Paired, hybrid vs hybrid-TTC and sfm vs
sfm-TTC: filter-active fraction, mean and minimum speed, stationary-time
fraction, stop-episode count and duration, time to goal among successes.
Supports the pathway only if motion changes *precede* the outcome.

**2. Temporal ordering.** For every deadlocked TTC run, compare
`t_first_activation`, `t_sustained_activation` and `t_deadlock_onset`. Deadlock
onset is the first step of the stall window, not its confirmation 5 s later —
ordering against activation is meaningless otherwise. **If deadlock frequently
begins before sustained activation, the filter cannot be the cause in those
runs**, and that is a falsification of the proposed pathway.

For averted contacts: did activation occur shortly before the point at which
the paired unfiltered trajectory made contact?

**3. Paired counterfactual trajectories.** Every scenario runs all four arms
deterministically, so within-simulation counterfactuals are available. Inspect:
hybrid unsafe but hybrid-TTC safe; hybrid reaching goal but hybrid-TTC
deadlocking; sfm vs sfm-TTC divergence. Strong comparative evidence, though not
a physical counterfactual.

**4. Outcome decomposition.** Under the mutually exclusive categories,

    Δ(safe goal) = −Δ(unsafe goal) − Δ(deadlock) − Δ(timeout)

Whether the safety gain comes from converting unsafe arrivals into safe ones,
or from converting arrivals into deadlocks, are different mechanisms with
different practical implications.

## A.3 Confounding caution — fixed in advance

**Do not compare runs where the filter activated against runs where it did
not.** Activation is *caused by* scenario difficulty and collision risk, so
that contrast is confounded by exactly the thing being studied. Only
randomised arm comparisons, paired scenario-level changes, temporal ordering,
and clearly-labelled mediation analyses are admissible.

## A.4 What may be claimed

After successful confirmation:

> TTC filtering produces a safety–mobility trade-off in the tested simulation:
> it increases deadlock and reduces goal completion, while improving safe-goal
> success when combined with A\* guidance.

With supporting mediator analysis:

> The trade-off is consistent with TTC-induced slowing and sustained stopping;
> A\* guidance appears to help the robot recover from safety interventions by
> supplying a feasible route.

**Not claimable without A.5:** that the mechanism is proven.

## A.4b Numeric definitions — frozen, so they cannot drift in analysis

| Term | Definition |
|---|---|
| stopped | robot speed < 0.05 m/s (`stallSpeed`) at a step |
| stop episode | a maximal run of consecutive stopped steps; duration = steps × dt |
| filter active | brake scale < 0.99 at a step |
| activation episode | a maximal run of consecutive active steps |
| sustained activation | first activation episode lasting ≥ 100 steps (1.0 s) — the timescale at which a brake becomes a stop rather than a momentary slow |
| deadlock onset | first step of the stall window, i.e. `terminationStep − 500 + 1`, NOT the confirmation step |
| unsafe event time | first step at which any swept surface gap ≤ 0 |

Per-run retained: first activation time, total activation duration and
fraction, activation episode count and durations, mean and minimum speed,
stopped-time fraction, stop episode count and durations, deadlock onset time,
goal-arrival time, unsafe-event time, minimum gap, outcome category.

## A.4c Status of the mediator variables

Arm assignment is randomised across paired scenarios, so the **outcome**
differences are causally attributable to adding the filter. The activation and
motion metrics are **post-treatment variables**: a scenario can independently
cause both elevated collision risk and slowing, so temporal order alone does
not establish that activation caused a stall.

Wording to use:

> Prespecified mediator measurements were used to assess whether the observed
> outcome changes followed the pathway predicted by the safety–mobility
> trade-off hypothesis.

**No mediated proportion is reported** unless a formal causal-mediation
framework is adopted and its assumptions defended. It is not, here.

## A.5 Dose–response manipulation — the actual causal test

Observation cannot establish the pathway; manipulating the proposed mediator
can. If intervention strength drives the trade-off, varying it should move
deadlock and safe success monotonically and in opposite directions.

**"Intervention strength" is not one axis.** `ttcHorizon`, `ttcStop` and
`ttcMinGap` have different mechanical consequences — a longer horizon gives
earlier but gentler braking, a higher stop threshold gives fewer but sharper
stops. Varying them jointly could produce non-monotonic activation even if the
mechanism holds. So each is swept **one factor at a time**, all others fixed:

| Factor | Levels | Immediate mechanical consequence |
|---|---|---|
| `ttcHorizon` | 1.0, 1.5, 2.0, 3.0 s | longer → activation earlier / over a broader state set |
| `ttcStop` | 0.4, 0.8, 1.2 s | larger → full stop demanded at greater predicted time-to-contact |
| `ttcMinGap` | 0.20, 0.40, 0.60 m | larger → separation floor constrains motion more often |

### Tier 1 — direct controller predictions

Near-mechanical consequences of the control law, and genuinely testable:

1. Activation occurs earlier, or over a broader set of states, as each factor strengthens.
2. Commanded motion is more constrained during activation.
3. Speed reduction during comparable activation episodes increases.

### Tier 2 — system-level predictions

Expected **on average**, not strictly:

4. Stationary time and deadlock risk increase.
5. Ordinary goal completion decreases.
6. Safe-goal success shows an **interior optimum**.
7. Unguided control reaches the mobility-cost-dominant region at a **weaker**
   setting than guided control.

**Prediction 7 is the sharpest test.** It is what the guidance-recovery
explanation requires and is not implied by any account in which the filter
simply makes the robot uniformly safer.

### Falsification, stated carefully

Failure of **Tier 1** would falsify the activation-to-slowing pathway, since
those are close to direct consequences of the algorithm.

Failure of **Tier 2** would not, by itself. Robot–crowd dynamics are nonlinear
and a stronger early intervention can sometimes resolve a conflict sooner,
reducing later activation or deadlock. Non-monotonic exceptions at the system
level do not refute the underlying mechanism.

---

# Amendment B — decision-branch labelling

**31 July 2026, while confirmation was executing and before any outcome was
inspected.** No computed quantity changed; only a printed label.

Branch verification with synthetic intervals showed the four decision branches
are mutually exclusive and correctly ordered — the zero test precedes the
threshold test, so `[-0.010, +0.120]` classifies as "includes zero" rather
than "spans the threshold", as required.

The check also exposed a gap: an interval lying **entirely below zero** was
labelled "did not replicate", conflating a *harmful* effect with an *absent*
one. A fifth branch now reports it as a reduction in safe-goal success.

`py/analyze_confirm.py`: `6e587a08857a66d9` → `a6f2d566523ede3c`.

*Correction: this line first carried a placeholder written before the
hash was computed — the same error made in Amendment A. Both are recorded
rather than overwritten. Hashes are now computed and substituted
programmatically rather than typed, so the failure mode is removed rather
than merely noted.*

Note on branch 1: "entirely above 0.075" requires the LOWER confidence bound to
exceed the threshold. It answers "can we conclude the true benefit exceeds 7.5
points?", which is deliberately stricter than observing a point estimate above
it. An interval whose lower bound equals 0.075 exactly falls to branch 2.


---

# Amendment C — outcome-transition decomposition

**31 July 2026, while confirmation was executing. No confirmatory outcome had
been accessed: no result file existed at the time of writing.**

Amendment A.2 item 4 planned the AGGREGATE outcome decomposition

    d(safe) = -d(unsafe) - d(deadlock) - d(timeout)

which shows net movement between categories but not which scenarios moved
where. Two filters with identical aggregate deltas can behave completely
differently — one rescuing 40 unsafe arrivals while breaking 30 safe runs looks
identical to one rescuing 10 and breaking none.

`py/transition_table.py` (`147c2e39d69d0f47`) reports the full 4x4 paired transition matrix
within identical scenarios, for hybrid->hybrid-TTC and sfm->sfm-TTC, with a
per-condition breakdown.

**Status, stated precisely:** an exploratory outcome-transition analysis
specified and frozen before confirmatory outcomes were available, extending the
aggregate decomposition defined in Amendment A. It was not part of the original
frozen protocol and will never be presented as such.

| Transition | Reading |
|---|---|
| unsafe -> safe | genuine rescue |
| unsafe -> deadlock | safety bought by abandoning progress |
| safe -> deadlock | harmful overconstraint |
| deadlock -> safe | improved recovery |
| safe -> unsafe | safety lost |

## Analysis order — fixed, no further changes

    confirmation -> primary -> secondary trade-off -> transition
    -> mediators -> dose-response -> recovery-controller experiment

The dose-response sweep and any recovery-controller experiment run only after
all confirmatory analyses are complete and archived. Adding a recovery
mechanism now would introduce a new intervention after confirmation had begun.
A recovery rule must be defined before its results are viewed.

**Analysis logic is now closed.** No further changes to definitions, thresholds
or decision rules.


---

# Amendment D — dose-response sample size and onset rule

**31 July 2026. Written BEFORE any dose-response run; no dose-response result
file existed at the time of writing.**

Amendment A.5 froze the factors, their levels, and the Tier 1 / Tier 2
predictions. It did not fix the sample size or state operationally what
"reaches the mobility-cost-dominant region" means. Both are fixed here, before
execution, because deciding either after seeing the sweep would let the result
choose its own test.

## Sample size

All **500 confirmatory scenarios**, both planners, 9 configurations
(1 filter-disabled reference + 8 unique one-factor-at-a-time settings) =
**9000 runs**. Every comparison is paired within scenario.

Reusing the confirmatory scenarios is deliberate and carries no confirmatory
status: this sweep manipulates the intervention rather than estimating the
frozen primary endpoint, and its results are labelled exploratory throughout.
The `ttcHorizon=2.0` cell reproduces the frozen default setting and therefore
acts as a validity check against the confirmatory `hybrid-ttc` arm.

## Onset rule for prediction 7

Prediction 7 requires the unguided arm to reach the mobility-cost-dominant
region at a **weaker** setting than the guided arm. Operationally:

> **Onset** = the smallest swept level of a factor at which that arm's
> goal-arrival rate (any arrival, safe or unsafe) falls to **<= 0.75x** the
> same arm's filter-disabled goal-arrival rate.

The criterion is **relative, not absolute**, because the two arms' baselines
differ by roughly fourfold (guided ~98%, unguided ~23% goal arrival). Any
fixed percentage-point threshold would be a different test for each arm and
would favour prediction 7 by construction.

Goal arrival is the mobility measure. Safe success is deliberately NOT used
here: it mixes mobility with safety, so an onset defined on it could not
distinguish the two.

If no swept level reaches onset for an arm, the result is reported as **"not
reached within the swept range"**. That is an honest null, not evidence for
either direction, and must not be converted into a claim about which arm
reaches onset first.

## Analysis rules

- Monotonicity across levels reported as Spearman rho with the level value. No
  significance threshold governs any claim.
- Paired percentile bootstrap within scenario against the filter-disabled
  reference for every rate.
- Tier 1 and Tier 2 are reported separately, with A.5's asymmetric
  falsification logic stated alongside: Tier 1 failure falsifies the
  activation-to-slowing pathway; Tier 2 failure does not, by itself.

## Instrumentation

`mean_speed_active` and `mean_brake_active` (mean robot speed and mean
commanded brake factor **conditional on the filter being active**) were added
to `py/sfm_explore.py` for Tier 1 prediction 3, which cannot be tested from
unconditional means.

Verified observational before use: 24 paired runs (6 scenarios x 2 planners x
filter on/off) compared field-by-field against unmodified `py/sfm.py` across
all 20 shared `Result` fields — **0 exact mismatches**. The accumulators read
state after it is committed and never feed back into velocity, position or
control flow.

`py/dose_response.py`: `a1f69cdb3def13e7`
`py/sfm_explore.py`: `a3639091ca7c2687`


## D.1 — labelling note (no computed quantity changes)

**31 July 2026, before any dose-response result was inspected.** Recorded
separately rather than edited into D, whose hashes are frozen against a run
already in progress.

The sweep reuses the 500 confirmatory scenarios. It must therefore be reported
as a **paired follow-up experiment on the confirmed scenario set**, and never
as another independent replication. It is efficient for testing the predicted
response pattern precisely because it is paired within the same scenarios, but
that pairing is exactly what removes its independence. Required phrasing:

> a paired follow-up experiment on the confirmed scenario set

Not: "replicated in a further 500 scenarios", or any wording implying a second
independent sample.

Two structural checks were added to `py/analyze_dose.py` (written before the
sweep produced output, and not frozen in D):

1. The nine configurations are asserted to be one-factor-at-a-time, forming
   THREE ordered scales sharing a common centre (`ttcHorizon=2.0` is the
   default setting), never one pooled 9-level "strength" scale. Spearman rho
   is computed within a factor only. If the levels were bundles, level k+1
   could not be claimed mechanically more restrictive than level k, and the
   configurations would have to be reported categorically.
2. The filter-disabled cell and the `ttcHorizon=2.0` cell must reproduce the
   archived confirmatory arms exactly (deterministic replay, tolerance 1e-9).
   Any drift means the sweep is not measuring the system the confirmation
   measured, and is investigated before any number is interpreted.

`py/analyze_dose.py`: `0a2847d642841aaa`
`py/Test_AnalyzeDose.py`: `8a468de33ad14f30`
`py/diversion_diagnostics.py`: `373f8dade86eb870`


## D.2 — analysis restructured into four evidence levels

**31 July 2026. The sweep was still executing; no dose-response output file
existed and none had been inspected.**

`py/analyze_dose.py` was rewritten after D.1 recorded its hash. The hash in D.1
is therefore superseded, and both are kept rather than overwritten. What
changed, and why:

1. **Four evidence levels, reported separately and never merged.** Previously
   "Tier 1 / Tier 2", which bundled behaviour and system response together and
   obscured that they fail independently:

   | Level | Claim | Effect of failure |
   |---|---|---|
   | L1 controller | parameter changes activation/braking as the control law requires | **falsifies** the activation-to-slowing pathway |
   | L2 behaviour | stopping, deadlock, goal completion move as predicted | does not refute L1 |
   | L3 system | safe success shows the trade-off / turning point | does not refute L1 or L2 |
   | L4 architecture | mobility-cost onset differs guided vs unguided | does not refute L1-L3 |

   Braking may rise monotonically at L1 while safe success stays noisy at L3.
   That is nonlinear robot-crowd dynamics, not refutation, and the report says
   so beside the result.

2. **Spearman no longer carries any claim.** With only 3-4 levels per factor a
   rank coefficient is a direction summary. Every level is now also reported as
   a raw estimate with a bootstrap interval, so the curve is visible rather
   than its sign alone.

3. **An interior optimum must be separated from the endpoints** by more than
   the intervals to be called one. Otherwise it is a ranking of noise, and is
   labelled "NOT separated from endpoints".

4. **Completeness and missingness are reported**, not assumed: expected cell
   count against observed, and any configuration not covering every scenario.

No frozen quantity changed: the factor levels, the onset rule, the 0.75x
relative criterion and the predicted directions are all as fixed in A.5 and D.

`py/analyze_dose.py`: `9acb679ff0f0ab91` (supersedes the hash in D.1)
`py/Test_AnalyzeDose.py`: `e6bf92140d9fc6f9` (updated for the new
output format; negative control re-verified — the test fails with the
`rate_goal` defect reintroduced and passes with it removed)
`py/diversion_verdict.py`: `fec9966815b9f588`


## D.3 — dose-response analysis: three-version audit trail

**1 August 2026, after the sweep completed.** The dose-response analysis must
be cited as **post-run corrected analysis**, NOT as untouched frozen output.
Three distinct versions exist and are kept separate:

| # | Digest | Status |
|---|---|---|
| 1 | recorded in D.1 | four-tier structure; **never executed on real data** |
| 2 | `9acb679ff0f0ab91` | four-level restructure (D.2); archived, then **aborted on its own assertion — produced no output** |
| 3 | `214ed9d6f99c08ea` | shared-centre lookup corrected; **produced every reported number** |

### What the correction did

`dose_response.py` runs the three sweeps' shared centre ONCE, labelled
`ttcHorizon=2.0`, rather than simulating the same 500 scenarios three times.
Version 2 looked up `ttcStop=0.8` and `ttcMinGap=0.4`, which exist as parameter
settings but not as labels. Its one-factor-at-a-time assertion caught this and
aborted. With the assertion bypassed it would have printed "configurations
missing — skipped" four times and reported **one factor of three**.

### Did the correction change numbers?

Measured, not asserted, by running the reconstructed pre-fix version with the
assertion bypassed and comparing against version 3:

- **Point estimates that differ: 0 of 32.** No computed quantity changed.
- **Bootstrap intervals that differ: 21 of 32**, all in the third decimal
  (e.g. `[0.186, 0.213]` -> `[0.186, 0.214]`).
- **Estimates available only in version 3: 48** — the two factors version 2
  could not read.

The interval drift is **not** a change of method. A single module-level
`default_rng` is shared across every bootstrap call, so interval endpoints
depend on how many draws were consumed earlier — and version 3 computes the
two extra factors first. This is a real reproducibility defect: an interval
should not depend on execution order.

**It is recorded rather than repaired retroactively.** Re-running to shift
third decimals would add another version to the audit trail for no scientific
gain. Future analyses (starting with the reciprocal-pedestrian study) must
derive a per-metric generator from a stable key so intervals are invariant to
execution order.

### Limitation carried forward, not fixed retroactively

A.5 prediction 3 requires "comparable activation episodes". The instrument
averages over an activation set that `ttcHorizon` and `ttcMinGap` themselves
define, so the conditional metric is interpretable only for `ttcStop`, where
the activation set moves by <= 3%. The completed sweep is **not** redesigned to
force comparability. Prediction 3 stands as supported where testable and
untested elsewhere; matched threat states belong in a future controller-level
study.

`py/analyze_dose.py`: `214ed9d6f99c08ea` (supersedes D.2)
`py/archive_followup.py`: `3c66c00959342e95`


---

# Amendment E — reciprocal (ORCA) pedestrian study

**1 August 2026. Frozen BEFORE execution; no reciprocal-pedestrian outcome
existed or had been inspected.**

## Question

Does the confirmed exposure mechanism depend on pedestrians that do not yield?
If reciprocal pedestrians largely remove the safe->unsafe degradation, then
`ttcMinGap`=0.2 may be compensating for a model-specific artefact rather than
revealing a broadly preferable controller setting.

## What varies, and what must not

The robot keeps its exact SFM + A* + TTC controller, integrator and
instrumentation. **Only the pedestrian model changes.** Driving the robot with
ORCA as well would change the intervention under test at the same time as the
pedestrian model, and no comparison could separate them.

SFM pedestrians feel a soft repulsion provably too weak to prevent contact
(goal drive 2.60 m/s^2 vs repulsion 2.10 at contact) and are pushed through.
ORCA pedestrians solve for a collision-free velocity and actively sidestep.
That is the qualitative change under test.

Implemented as an optional pedestrian-velocity hook in `py/sfm_explore.py`.
**Verified inert when unused**: 16 paired runs x 20 shared `Result` fields
against unmodified `py/sfm.py`, 0 mismatches.

## Scenario sets — and why NOT fresh seeds

**Correction, 3 August 2026.** This section originally read: *"MATLAB and
Octave are unavailable in this environment, so the original generator cannot
produce new seeds."* **That premise was false.** MATLAB R2025b was installed at
`/Applications/MATLAB_R2025b.app`; the availability check used `which matlab`,
which searches `PATH` only, and the application is not on `PATH`. The generator
could have been run at any time.

The design decision is **unaffected and stands on its own merits**: scenarios
are held fixed and the pedestrian model is the only thing that varies, so a
difference is attributable to the model rather than to scenario sampling. Fresh
seeds would have confounded the two, and would have been the wrong choice even
had the constraint never been believed.

What must be corrected is the *justification*: the design was presented as
coinciding with an environmental constraint. There was no constraint. Any
statement that new seeds were unavailable is withdrawn.

| Set | n | Role |
|---|---|---|
| `export_confirm` | 500 | primary; paired against archived SFM results scenario by scenario |
| `export_dev` | 300 | seed-disjoint from the primary set; separate check |

Four benchmark conditions unchanged (125 each in the primary set).

## Arms

`disabled` (TTC off) · `ttc0.40` (confirmatory default) · `ttc0.20` (the
setting the sweep favoured).

Responsibility: primary **`reciprocal`** (standard ORCA). The robot does not
run ORCA and so does not take its half, meaning pedestrians supply half the
avoidance of a true ORCA-ORCA pair — stated, not hidden. Secondary
**`ped_yield`** inflates the robot's radius by 0.20 m as pedestrians see it,
increasing clearance without mathematically reassigning responsibility.
**Neither is exact full-pedestrian-responsibility and neither will be described
as such.**

## Outcomes, fixed before running

**Primary:** `p_safe(ttc0.40) - p_safe(disabled)` under reciprocal pedestrians,
primary set. Does the confirmed guided benefit survive the model change?
Smallest meaningful effect +0.075; the same five decision branches as the
confirmatory analysis.

**Secondary, ordered:**
1. safe->unsafe transition count, `disabled` -> `ttc0.40`. Does the degradation
   persist when pedestrians yield? This is the mechanism test.
2. `p_safe(ttc0.20) - p_safe(ttc0.40)`. Does the floor preference generalise?
3. Both of the above on the disjoint development set.
4. Both under `ped_yield`.

## Interpretation rules

- Results here are **generalisation evidence**, NOT independent confirmation of
  the SFM parameter effect. `ttcMinGap`=0.2 was selected from the SFM sweep, and
  this study is neither designed nor powered to confirm it. An independent
  confirmation still requires disjoint seeds and no further parameter selection.
- A reduction in safe->unsafe degradation under ORCA pedestrians supports the
  mechanism being **amplified by non-yielding pedestrians**. Persistence
  supports it being **a general consequence of hard TTC braking**. Either is a
  publishable answer; neither is the hoped-for one.
- Bootstrap generators must be derived per metric from a stable key, so
  intervals do not depend on execution order (the defect recorded in D.3).

## Validity gate, executed before freezing

ORCA pedestrians must demonstrably differ from SFM pedestrians, or the study is
void — the failure mode that made an earlier `maxSpeed=0` ORCA setting silently
inert. Checked on 3 scenarios: outcomes differ in steps and contact counts in
**3/3**. Gate PASSED; the pedestrian model is live.

`py/reciprocal.py`: `d2430a01ff3a500c`
`py/run_reciprocal.py`: `2ac0a97b3b0fc49c`
`py/sfm_explore.py`: `d841bf73089a885d` (pedestrian hook added; supersedes D)


## E.1 — naming, attribution, and parameter disclosure

**1 August 2026, while the study executed. No reciprocal outcome existed or had
been inspected. No computed quantity changes; this fixes naming and the
permitted attribution.**

### Attribution — what a difference may be ascribed to

Replacing SFM pedestrians with ORCA pedestrians does not vary one abstract
quantity called "yielding". It simultaneously changes interaction range,
anticipation, speed selection, reciprocal collision geometry, crowd
organisation, congestion propagation and local trajectory smoothness.

**Forbidden:** "any difference is caused specifically by pedestrian yielding."

**Required:** *any paired difference is attributable to replacing the SFM
pedestrian dynamics with the specified ORCA-based pedestrian model.*

Whether increased pedestrian avoidance is the plausible explanation is then
assessed from activation, stopping, duration and transition analyses — as
evidence, not as a definition.

### Arm naming

| Use | Not |
|---|---|
| SFM pedestrians | — |
| ORCA-responsive pedestrians | "reciprocal navigation" (unqualified) |
| ORCA-responsive pedestrians with inflated robot radius | "full responsibility" |

The pedestrians respond reciprocally **within their own model**, but the robot
does not share an ORCA constraint. Unqualified "reciprocal" overstates it.

### Sample status

The frozen confirmatory scenarios supply **paired initial conditions** —
geometry, obstacles, goals and starting states — while the outcomes under
ORCA-responsive pedestrians are newly generated. Required phrasing:

> The study uses the frozen confirmatory scenarios as paired initial conditions
> but introduces newly generated outcomes under ORCA-responsive pedestrian
> dynamics.

These 500 scenarios are **not** an independent sample relative to the original
confirmation. The 300 development scenarios are a **robustness set only** — they
informed earlier design work and cannot serve as a second confirmatory sample.

### Parameters, disclosed

Inherited from `SfmParams`, identical to the SFM arm: dt 0.01 s; pedestrian
radius 0.30 m; robot radius 0.35 m; pedestrian preferred speed 1.34 m/s; max
speed 2.0 m/s; goal tolerance 1.0 m. Obstacles are the same polygons, wound
counter-clockwise for RVO2 solidity. Pedestrian goals are maintained by the SFM
engine and passed in; ORCA sets only the PREFERRED velocity toward them, so
goal sequencing and arrival behaviour are identical across pedestrian models.

ORCA-specific: neighbour distance 5.0 m (matching SFM `rInfPed` = 5.0, so the
interaction range is not silently altered); max neighbours 10; agent time
horizon 2.0 s; obstacle time horizon 1.0 s; robot inflation 0.20 m under
`ped_yield` only.

**These choices materially control how much avoidance occurs** and are reported
as design parameters, not as neutral defaults. No sensitivity sweep over them is
performed or implied.

### Role of ttcMinGap = 0.20

Frozen in E as secondary outcome 2 and analysed only as such. It supplies
**external-model evidence for a previously selected candidate**. It does not
erase the need for a fresh, independent, seed-disjoint SFM parameter
confirmation with no further parameter selection, and must not be promoted on
the strength of this study.
