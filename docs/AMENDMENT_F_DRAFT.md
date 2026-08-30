# Amendment F — TERMINATED 6 August 2026 (negative calibration result)

## DECISION NOTE

> Stage B satisfied the prespecified calibration continuation rule because at
> least one candidate reduced deadlock by at least 3 percentage points while
> remaining within the loose calibration screens. However, no candidate
> simultaneously met the final design targets of no more than a
> 2.5-percentage-point increase in unsafe arrival and at least a
> 5-percentage-point reduction in deadlock. Because the protocol did not
> specify a selection rule for this empty feasible set, Amendment F was
> terminated without formal evaluation. No thresholds, gates, or
> candidate-ranking rules were changed after observing Stage B.

**This is not a waived rule.** The frozen continuation rule answered only
whether there was enough deadlock reduction to justify *considering*
evaluation. It never established that a candidate could satisfy both final
gates. Stage B supplied information the rule did not anticipate: the feasible
set is empty. Proceeding because a loose procedural threshold technically
passed would elevate that threshold above the stated engineering objectives.

**Options rejected, with reasons.**

*Evaluate the rule-selected candidate.* It sits at $+4.4$ pp unsafe against a
$+2.5$ pp margin. Failure is not mathematically certain, but powering a large
study around a candidate already pointed well past the boundary is poor use of
remaining effort, and would read as: *we selected a controller that failed the
safety criterion in calibration, then powered a study to see whether it might
pass anyway.*

*Evaluate the best Gate-1-compliant candidate.* Worse methodologically: it
replaces the frozen ranking with "choose the best apparently compliant
candidate", a rule invented after seeing Stage B. Its $-2.5$ pp is half the
worthwhile threshold, so even a safety pass would leave the intervention
ineffective by its own efficacy criterion.

## Scope of the negative result — do not overstate it

**Permitted:** *this progress-aware release design did not resolve the
trade-off.*

**Forbidden:** *recovery cannot resolve the trade-off.* One mechanism was
tested, over one grid, on one calibration set. Other recovery architectures
(replanning, lateral biasing, progress-aware waypoint advancement) are untested
here and are not addressed by this result.

## Why it strengthens rather than weakens the paper

The recovery layer reproduces the paper's central trade-off one level up: the
controller cannot recover substantial mobility by releasing the discretionary
brake without reintroducing unsafe motion. Under this design the trade-off is
structural, not an artefact of a single badly chosen TTC threshold.

---

# Frozen record (retained)



## Stage B — frozen before execution

**Triggers, chosen by the ranking frozen before Stage A ran** (not for
diversity, which was never a written criterion):

    Trigger 1  w4.0_p0.3_a0.8   (rank 0)
    Trigger 2  w4.0_p0.2_a0.8   (rank 1)

**Grid:** release {0.4, 0.6, 0.8} x max duration {2, 3, 5} s x episode cap
{1, 2, 3}, for both triggers = **54 configurations**. No finer levels are
added. If no candidate approaches the target, the conclusion is that the
architecture lacks leverage, not that the grid needs refinement.

### The trigger robustness check is near-vacuous, and must be reported as such

Trigger 1 and Trigger 2 differ in **1 of 160 scenarios**. `theta_prog` is
weakly identified over 0.10-0.30 m because route progress under sustained
braking is effectively bimodal: near zero, or well above every tested
threshold.

Running both preserves the written design and avoids making Stage B
outcome-adaptive. But **agreement between the two triggers is guaranteed by
construction and is NOT evidence of robustness.** Any statement that "the same
action wins under both triggers, strengthening the selection" would be
circular. The correct report is that the frozen ranking produced a
near-duplicate second trigger, so the intended robustness stratum could not be
exercised.

The selected `p = 0.30` is **tie-break selected, not empirically optimal**.

### Reporting rules, fixed before execution

1. **Separate trigger and action sensitivity.** For every action
   configuration, report the outcome under Trigger 1, under Trigger 2, and
   their difference.
2. **Verify non-inert action variation before interpreting any ranking.**
   Release fraction, duration and cap must each change at least one of: total
   released-brake time, episode count, final applied scale, or the outcome in
   at least one scenario. `theta_prog` already failed an equivalent test; the
   action parameters must be checked rather than assumed.
3. **Gates are not relaxed.** Gate 1 +2.5 pp, Gate 2 -10 pp, Gate 3 -5 pp
   stand. The Stage A shortfall is an early warning, not grounds for revision.

## FUTILITY RULE — frozen NOW, before Stage B runs

> Do not launch the formal evaluation if no Stage B candidate reduces
> calibration deadlock by at least **3 percentage points** while remaining
> within the calibration safety and rescue screens.

Three points sits deliberately below the 5-point evaluation gate, allowing for
sampling uncertainty on 160 scenarios while preventing an expensive evaluation
of an evidently ineffective controller.

**This rule is frozen before Stage B outcomes exist so that it cannot be
invented, or waived, afterwards.** Stage A's best candidate achieved 2.5
points, so failure is a live possibility and the rule is not a formality.

### Timing of this rule — state it exactly, do not backdate it

**The futility rule was added AFTER Stage A and BEFORE Stage B.** It was not
part of the original Amendment F design and must never be described as such.
Required wording:

> After Stage A revealed a 2.5-point maximum calibration deadlock reduction,
> and before any Stage B outcome existed, a continuation criterion of at least
> 3 percentage points was added to prevent formal evaluation of an evidently
> weak controller.

This is development-stage adaptation, acceptable because Stage A is calibration
rather than evidence — but the timing is what makes it acceptable, so the
timing is reported.

**Three points was not chosen to let Stage A pass.** Stage A's 2.5 points does
NOT satisfy it. The threshold sits below the 5-point evaluation gate to allow
for sampling uncertainty at n = 160, and above the observed Stage A result so
that it constrains rather than ratifies.

### Reporting details

- The **56 configurations are 54 recovery candidates plus 2 reference arms**
  (no-filter and TTC-only). State this, or the count appears to contradict the
  documented 2 x 3 x 3 x 3 = 54 grid.
- Report **raw scenario counts beside percentages**. At n = 160 one scenario is
  0.625 percentage points, so the 3-point continuation rule requires at least
  **five net scenario conversions** — a fact that percentages alone conceal.
- **Trigger 2 is not a robustness validation.** The honest conclusion is that
  Stage A failed to identify two meaningfully distinct top triggers.

### If F stops

Close it as a negative engineering result:

> Progress-aware relaxation of the discretionary TTC brake was implemented and
> calibrated, but no candidate achieved the prespecified minimum calibration
> efficacy while preserving safety; formal evaluation was therefore not
> launched.

Do not revise the grid, margins, trigger, emergency override or futility
threshold in response. Stronger than omitting recovery silently, and stronger
than forcing an underperforming design through a powered evaluation.



All five decision values are now fixed, **before Stage A was executed and before
any recovery outcome was inspected**. They may not be revised afterwards.

| Symbol | Value | Justification |
|---|---|---|
| calibration unsafe tolerance | **+0.050** | Screening guardrail only. The 160-scenario calibration set has ~0.6 pp resolution; a 2.5 pp screen would be finer than the data and would overreact to a handful of cases. NOT evidence of safety preservation. |
| calibration rescue floor | **−0.150** | Reject candidates losing more than 15 pp of rescue rate on the calibration unsafe pool. |
| **Gate 1** unsafe-arrival NI margin | **+0.025** | Recovery deliberately relaxes a safety intervention. More than 2.5 pp would consume a substantial share of the confirmed 19.8 pp unsafe-arrival reduction. |
| **Gate 2** rescue NI margin | **−0.100** | Permits some mismatch; rejects a controller discarding more than one in ten TTC rescue opportunities. |
| **Gate 3** minimum deadlock benefit | **−0.050** | The confirmed TTC increase was 12 pp; 5 pp removes ~42% of the added burden. Less would not justify the added state machine, thresholds, logging and failure modes. |
| target power | **0.90** | For Gate 1 at 2.5 pp AND for detecting a 5 pp deadlock reduction; the larger required N governs. |

Power planning uses calibration discordance estimates, but **simulates
conservative upper and lower discordance cases** rather than a single point
estimate.

---



**A safety-gated paired experiment testing whether progress-aware release of
the discretionary TTC brake reduces TTC-induced deadlock while preserving TTC's
safety benefit.**

Scope is deliberately narrow. F modifies **the TTC safety layer only**. It does
not touch global-route recovery, which is a separate architectural question
(see "Relationship to §5a").

Not frozen. Every `TBD_` below needs a written engineering justification, and
no threshold may be chosen after seeing any recovery outcome.

## Target

The one failure robust across every model tested:

    sustained TTC intervention -> loss of progress -> deadlock

SFM 2.0% -> 14.0%; ORCA-responsive 2.0% -> 21.6%; ORCA-responsive + inflated
radius 2.2% -> 25.0%. Contacts are deliberately NOT the target: that result
reverses with a 0.20 m radius change and turns on sub-1.4 mm boundary
crossings.

## Relationship to §5a — no interaction, because the mechanism is absent

A code audit (3 Aug 2026) established that PROTOCOL §5a blockage handling was
**never implemented** in either engine. There is therefore nothing to enable,
disable or balance across arms.

**This must be stated in the manuscript rather than left implicit**: the reason
there is no confound is that the mechanism does not exist, not that it was held
constant by design.

**Required arm statement:**

> All arms use identical strict one-shot A\* paths, sequential waypoint
> progression, and no event-triggered waypoint skipping or replanning. The sole
> difference between hybrid–TTC and hybrid–TTC–recovery is the progress-aware
> relaxation of the discretionary TTC brake.

The §5a three-way ablation (strict one-shot / safe waypoint advancement /
event-triggered A\* recovery) stays **outside F** as a separate outstanding
commitment. Folding it in would turn a clean contrast into at least a 3x3
architecture study and would leave deadlock improvements unattributable between
brake release, waypoint skipping and replanning.

## Populations

**Primary: SFM pedestrians.** TTC has a confirmed net benefit there and both
mechanisms are measurable.

**The pool is defined prospectively.** The historical confirmatory set
contained 314 unfiltered-unsafe scenarios (62.8%). In the new evaluation set,
`U` is defined from that set's own unfiltered hybrid arm and **its size is
unknown before execution**. The historical figures are power-planning inputs
only and must never be quoted as the future pool size.

**Robustness: both ORCA-responsive variants.** Does recovery reduce
TTC-induced deadlock without increasing meaningful penetration severity under
alternative pedestrian dynamics? Rescue pools there are 45 and 9 — too small
for a formal criterion, so rescue retention is **descriptive only** under ORCA.

## Arms

    hybrid  |  hybrid-TTC  |  hybrid-TTC-recovery

The unfiltered `hybrid` arm is a **reference**, included to reveal whether
recovery merely approximates "TTC off". It does **not** need to be dominated on
every metric. The engineering goal is to preserve TTC's meaningful safety
protection while reducing its deadlock and mobility cost.

## Ordered decision rule

Gates run in order; a later gate is interpreted only if the earlier passes.

### Gate 1 — safety preservation

Recovery is non-inferior to standard TTC on the **original unsafe-arrival
probability**, margin `TBD_MARGIN`.

One endpoint, one margin. The three-margin severity gate previously drafted is
withdrawn: it imported a pathology specific to the ORCA-responsive robustness
model (sub-1.4 mm grazes) into the SFM primary, required three justified
margins and a "severe contact" threshold that does not exist, and rested on a
heavily zero-inflated continuous endpoint.

**Reported but NOT gated:** maximum penetration, contact duration,
severe-contact incidence.

### Gate 2 — rescue preservation

Gates rather than reports, because it is what prevents "just disable TTC" from
scoring as a success.

**Estimand — a paired difference on a FIXED pool, not a ratio.** Define the
unfiltered-unsafe pool once, from the `hybrid` arm:

    U = { i : hybrid_i is unsafe }        (SFM: |U| = 314)

Both rescue rates are then estimated on that same pool, so every scenario
contributes to both arms:

    p_rescue,TTC = P(safe under standard TTC | i in U)
    p_rescue,REC = P(safe under recovery     | i in U)

**Gate:** non-inferiority of the paired difference,

    p_rescue,REC - p_rescue,TTC > -TBD_RESCUE

The earlier formulation gated on the ratio
`#(hybrid unsafe -> recovery safe) / #(hybrid unsafe -> standard TTC safe)`.
That is intuitive but **is not a simple paired estimand**: numerator and
denominator can count different scenarios, so the two counts are not a matched
comparison, and the quantity is undefined when the denominator is zero. The
paired difference on a fixed pool is statistically cleaner and handles
scenarios rescued by one controller but not the other by construction.

The retention **ratio is retained as an interpretable secondary summary only**.
It may exceed 1 when recovery rescues scenarios standard TTC did not; that is
legitimate and is reported untruncated. If `|U|` were ever 0 both rates are
inestimable and the gate is **skipped and reported as such**, never treated as
passed.

### Gate 3 — deadlock efficacy

    p_deadlock(recovery) < p_deadlock(TTC)  by at least TBD_DEADLOCK

Paired within scenario, exact McNemar plus bootstrap interval. A frozen minimum
worth the added controller complexity — a one-point reduction would not justify
a new control layer.

### Gate 4 — supporting outcomes, reported together, none gating

safe-goal success; total goal arrival; maximum penetration; contact duration;
sustained activation; stop time; and the complete transition decomposition
(unsafe->safe retained, unsafe->deadlock reduced, safe->deadlock reduced,
safe->unsafe not materially increased).

## ORCA robustness analyses

Lead with **continuous severity**, because the binary endpoint was shown to be
boundary-sensitive in that model. Separately defined tests or descriptive
reporting. **The ORCA knife-edge must not dictate the SFM primary endpoint.**

## Controller specification — all items fixed before any trial

### Progress coordinate — arc length on the FIXED initial polyline

F contains no replanning, so the path cannot change. Progress is the reduction
in remaining distance along the initial A\* polyline over the window:

    delta_s = s_remaining(t - W) - s_remaining(t)

Edge cases that must each have a stated rule:

**A monotone segment index was the first implementation and was WRONG.** It
blocked self-intersection jumps but silently saturated `s_remaining` when the
robot moved back across a passed waypoint, discarding exactly the signal the
trigger needs. External review caught it; the original tests missed it because
they exercised backward motion only WITHIN a segment.

The implemented rule:

> Candidate projections are restricted by consistency between implied
> arc-length change and physically observed displacement, permitting genuine
> backward motion while rejecting implausible jumps between spatially adjacent
> but route-distant segments.

| Case | Rule |
|---|---|
| robot passes a waypoint | adjacent segments share the vertex; `s_remaining` continuous |
| projection would jump to a route-distant segment | rejected: the arc change exceeds what the step's displacement allows |
| robot moves backward across a passed waypoint | admitted: displacement supports it; `s_remaining` rises by the full backward distance and progress goes negative, unclamped |
| path self-approaches | the plausibility bound rejects the far candidate |
| sampling assumption | `remaining()` must be called every simulation step; the bound is displacement-based |
| replanning | **cannot occur in F**. If §5a is ever implemented, progress must reset onto the new path under a separately defined rule. That complexity is deliberately NOT built now. |

### Remaining parameters

| Item | Draft | Status |
|---|---|---|
| required progress over window | 0.20 m / 3.0 s | Stage A screening default |
| minimum sustained activation | 80% of window | Stage A screening default |
| release magnitude | 0.60 of discretionary brake | Stage A screening default |
| maximum release duration | **3.0 s** | Stage A screening default |
| re-entry hysteresis | 1.0 s restored progress | Stage A screening default |
| cooldown / re-arm | **2.0 s of DEMONSTRATED restored progress, latched once earned** | FIXED structural safeguard, not calibrated |
| hard-gap / urgent-TTC override | absolute, never released | flags come from `ttc_brake_ex`; precedence tested end-to-end |
| episode cap | **2** | fixed for Stage A; enforced at TRIGGER time |
| after the cap | restore full TTC; ordinary 5 s stall rule classifies the trial | stated |
| replanning | **none** — brake release only | stated |

**Unit tests establish logic, not parameter values.** 39 tests now pass across
`test_path_progress.py` (12), `test_recovery_state_machine.py` (18),
`test_ttc_result_compatibility.py` (3) and `test_recovery_integration.py` (6).
They show the machine behaves as specified and that wiring it in perturbs
nothing it should not. They say nothing about whether the screening defaults
are appropriate values -- that is what calibration is for.

Two defects were found by these tests and would not have been found by
inspection: a re-arm counter that could never satisfy itself, and an episode
cap enforced one episode late.

Episode cap: the existing §5a text uses two blockage-recovery attempts, which
could justify consistency — but §5a was never implemented, and its attempts are
route-recovery events, not TTC-release episodes. **Do not silently conflate
them.**

## Calibration — set exists and is verified

**F calibration set**: 160 scenarios, 40 per condition, 0 rejected in 160
attempts. Collective sha256 `1dc1b1d2928f961f...`, manifest
`v2/runs/v2/fcal_manifest.json`. Seeds drawn from [2^31, 2^32-1]; every prior
seed lies below 2^31, so disjointness is structural. Verified 160/160 distinct
obstacle maps and crowd layouts, and 0 geometry shared with `export_confirm`.

*(An earlier draft recorded MATLAB as unavailable. That was false -- the check
used `which matlab`, which searches PATH only. Withdrawn; see ENVIRONMENT.md.)*

### Stage A — trigger screening, 18 configurations

Action held FIXED at the screening defaults above while only the trigger
varies:

| Factor | Levels |
|---|---|
| progress window | 2, 3, 4 s |
| required arc-length progress | 0.10, 0.20, 0.30 m |
| sustained activation fraction | 0.6, 0.8 |

3 x 3 x 2 = 18. Plus two reference arms (`hybrid`, `hybrid-TTC`) on the same
160 scenarios.

### Ordered selection rule — frozen BEFORE running

Not "maximise safe success". Ordered constraints:

1. **Reject** any configuration raising unsafe-arrival rate above standard TTC
   by more than the provisional calibration tolerance `TBD_CAL_TOL`.
2. **Reject** any configuration materially losing TTC rescues on the fixed
   calibration unsafe pool.
3. Among survivors, choose **lowest deadlock**.
4. Tie-break: fewer recovery episodes.
5. Then: shorter total release time.
6. Then: the weaker intervention -- lower release fraction, shorter maximum
   release, fewer allowed episodes.

Rules 4-6 penalise unnecessary complexity and aggressiveness.

**The calibration tolerance is NOT the final Gate 1 margin.** Calibration only
chooses parameters; the evaluation margins are justified independently.

### Stage B — action, on the best two triggers only

release {0.4, 0.6, 0.8} x max release {2, 3, 5} s x episode cap {1, 2, 3}
= 54 configurations.

### Execution and archiving

All three arms rerun on all 160 scenarios -- archived outcomes are NOT reused,
so the manifest is self-contained for the exact code version. Archive before
inspecting: scenario manifest hash, controller code hashes, grid definition,
selection rule, raw outcomes, transition tables, recovery traces, crash and
missing-cell counts.

### Evaluation set

Exported only AFTER power analysis fixes N. Same four-condition balance, a
further disjoint seed family.

## Current code hashes

    py/ttc.py            f7ea7f6093296ffc
    py/recovery.py       02707d64c39fc625
    py/path_progress.py  448bed0a0d90b2e2
    py/sfm_explore.py    1d853348e8bf237e

## Constraints

- Do not modify the archived ORCA-responsive conclusions.
- Do not add a robot ORCA arm.
- Do not fold in the §5a ablation.
- Do not tune any threshold after viewing recovery outcomes.
