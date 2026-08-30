# Amendment G2 — path-invalidation experiment

**Frozen 6 August 2026, before any scenario is generated.** Supersedes the
retired Amendment G blockage suite.

## Why G was retired and G2 is different

G tried to create route-recovery conditions by making geometry *harder* and
hoping the baseline would fail. It did not: 0 deadlocks in 75 calibration
scenarios. The temptation was then to narrow gaps until deadlocks appeared,
which would have tuned the benchmark until it produced the phenomenon under
study.

G2 instantiates the reviewer's premise **by construction** instead. The route
is invalidated as a *geometric fact*, not as an observed outcome:

1. compute the initial A\* route;
2. at a frozen trigger, change the static map so the active route segment
   becomes invalid;
3. verify by geometry that the original remaining path is now blocked and that
   a collision-free alternative exists;
4. give all three arms the identical map update; only arm C may replan.

The contrast is then clean and does not depend on tuning severity:

| Arm | After the map change |
|---|---|
| **A** | retains the invalid original route |
| **B** | may advance along the original route; cannot create a new one |
| **C** | may advance, and may replan against the updated static map |

## Families — static-map changes only

Pedestrian-held blockage is **removed from the primary comparison**. Arm C does
not plan around people, so those families cannot isolate replanning. They may
reappear as an appendix diagnostic about waiting and advancement, never as part
of the replanning claim.

| Family | Map change |
|---|---|
| `corridor_closed` | the corridor carrying the active route segment is sealed |
| `waypoint_enclosed` | a new obstacle encloses the active waypoint |
| `wall_displaces` | a wall appears between the robot and the remaining route |
| `passage_switch` | the original passage closes while a second passage remains open |
| `pocket_formed` | a concave route segment is invalidated, with a valid route around the exterior |

## Validity conditions — ALL required, checked geometrically

A scenario is accepted only if every one holds:

1. an initial A\* path exists **before** the map change;
2. the **remaining** original path is invalid **after** the change (it
   intersects the new obstacle);
3. a collision-free alternate path to the goal exists after the change;
4. the alternate route differs **materially** from the original (Hausdorff
   distance $\geq 3$ m, so a trivially perturbed path does not qualify);
5. obstacle-overlap and endpoint-clearance rules pass, as in G;
6. the map change is **independent of arm behaviour**;
7. the change trigger --- time or route progress fraction --- is frozen per
   scenario by its seed;
8. all arms receive the identical updated static map;
9. **no scenario is accepted or rejected on the basis of whether arm A
   eventually deadlocks.**

Condition 9 is the one that matters most. Using "arm A must fail" as a
generation filter would rebuild exactly the error that retired G: geometry, not
observed outcome, establishes route invalidation.

## Size — deliberately small

| Pool | Per family | Total |
|---|---|---|
| calibration | 10 | 50 |
| evaluation | 30 | 150 |

Three paired arms, one shared parameter set across all families. Seeds disjoint
from every prior family, including the retired G pools.

## Outcomes

**Manipulation check, reported FIRST, before any arm comparison:**

> After the map change, the original remaining route was invalid in 100% of
> accepted scenarios, while a valid alternate A\* route existed.

If that does not hold, nothing else in G2 is interpretable.

**Primary:** safe goal completion, arm A vs arm C, paired within scenario.

**Secondary, ordered:** deadlock; unsafe arrival/contact; route validity at
termination; replanning success rate; traversal time; path length; computation
time. A vs B isolates advancement; B vs C isolates replanning.

**Reported pooled AND per family.** The method is never selected on its best
family.

## Pre-trigger recovery — measured, not suppressed

Arm C may replan BEFORE the map change, because the frozen arm C rule fires on
any progress stall and says nothing about the trigger. A robot stalled by crowd
congestion genuinely satisfies the condition.

Measured on the 50-scenario calibration pool:

| stall / progress | scenarios with a pre-trigger replan | pre-trigger / total replans |
|---|---|---|
| 2.0 s / 0.50 m | 3/50 | 3/99 |
| 3.0 s / 0.30 m | 1/50 | 1/89 |
| 4.0 s / 0.10 m | 0/50 | 0/77 |

**Decision: keep the frozen rule; report pre-trigger replans as a measured
covariate.** Restricting arm C to post-trigger replanning would give a cleaner
contrast, but it would change the arm definition after observing behaviour, and
the contamination is 0-3% of replans. Selecting trigger parameters to minimise
it would add a criterion the selection rule does not contain.

Every G2 result reports the pre-trigger replan count alongside the total, so a
reader can bound the contamination rather than take our word for it.

### Required description of arm C

**Permitted:**

> Arm C is a general event-triggered recovery controller evaluated in
> path-invalidation scenarios; a small fraction of replans occurred before the
> map change due to ordinary congestion, and these were measured explicitly.

**Forbidden:** describing arm C as a *post-invalidation replanner*. It is not.
It fires on any progress stall, and 0-3% of its replans preceded the map
change.

## Calibration selection rule — frozen before the screen runs

Grid: `G_STALL` in {2, 3, 4} s x `G_PROG` in {0.10, 0.30, 0.50} m = 9
configurations, applied identically to arms B and C. Arm A is
parameter-independent and is run once per scenario.

Ordered constraints, mirroring Amendment F's structure:

1. **Reject** any configuration whose arm-C unsafe-arrival rate exceeds arm A's
   by more than **+5.0 points** (calibration screen; NOT the evaluation
   margin).
2. Among survivors, choose the highest **arm-C safe goal completion**.
3. Tie-break: fewer replans, then fewer advancements, then the **weaker
   trigger** --- larger `G_STALL`, then larger `G_PROG`.

Rule 3 penalises unnecessary intervention, as in Amendment F.

**Rule 1 is a SCREENING rule at n = 50, not a non-inferiority test.** It must
never be reported as evidence that arm C preserves safety. A formal
non-inferiority claim would require the evaluation pool and a prespecified
margin, neither of which this screen provides.

## Reporting order — fixed, for both calibration and evaluation

1. manipulation validity (must be 100% before anything else is interpreted)
2. parameter-screen result
3. selected shared trigger
4. pre-trigger contamination frequency
5. pooled outcomes
6. family breakdown
7. component contrasts

**Component contrasts are the informative ones**, and are always reported:

| Contrast | Isolates |
|---|---|
| B vs A | the value of waypoint advancement alone |
| C vs B | the added value of replanning |

A vs C alone would conflate the two mechanisms.

The pre-trigger replan count is a **reported diagnostic in both calibration and
evaluation**, never dropped once calibration is done.

**Evaluation scenarios are never rejected or regenerated on the basis of arm
behaviour.** The pool is generated once, validated geometrically, and touched
once.

**There is no "arm A must fail often enough" continuation rule.** The
manipulation check is the validity gate. If arm A still succeeds despite route
invalidation, that is a legitimate result and the scenario remains
construct-valid.

## Recovery parameters

Inherited unchanged from the retired G design, which was frozen before any
outcome was seen and is not implicated in the construct-validity failure:
`G_CLEAR` 0.50 m, `G_MAXSKIP` 3, `G_COOLDOWN` 3.0 s, `G_MAXADV` 3,
`G_MAXPLAN` 2. Only `G_STALL` and `G_PROG` are calibrated, on the 50-scenario
G2 calibration pool.

## Carried over unchanged

Arm B's advancement rule, arm C's replanning rule, and the static-map-only
perception constraint are exactly as specified in the retired G document. Only
the scenario construction changes.

---

# G2 CALIBRATION OUTCOME — NEGATIVE. Evaluation pool NOT launched.

**Recorded 30 August 2026** from `v2/runs/v2/g2_calib.json` (950 runs, 63.9 min)
via `py/analyze_g2_calib.py`. Full output: `v2/runs/v2/g2_calib_output.txt`.

## 1. Manipulation check — PASSED, independently re-verified

**200/200 (100%)** across both pools — 50 calibration and 150 evaluation.
Re-derived from each scenario's stored geometry using segment-polygon and
Hausdorff code written independently of the generator; the generator's own
stored boolean flags were deliberately **not** read.

| Pool | Pass | Hausdorff median | min | max |
|---|---|---|---|---|
| calibration | 50/50 | 6.0 m | 3.0 m | 28.4 m |
| evaluation | 150/150 | 5.8 m | 3.0 m | 28.4 m |

G2's construct is sound. The route is genuinely invalidated and a materially
different valid alternative genuinely exists.

## 2. Parameter screen — NO configuration passed

Arm A: safe **0.000**, unsafe 0.080, deadlock **0.920**.

All nine arm-C configurations exceed the +0.05 unsafe tolerance
(`d_unsafe` +0.16 to +0.28). The screen rejects the full grid.

## 3. The primary endpoint is degenerate — the decisive finding

**Safe goal completion is 0.000 for arm A and for all 18 arm-B and arm-C
configurations. Zero of 950 runs ended in a safe goal arrival.**

The prespecified primary comparison — safe completion, arm A vs arm C, paired
within scenario — **cannot be made on this pool**. This holds independently of
the safety screen and would still hold if the screen were removed.

| Arm | safe | unsafe | deadlock | replans | adv |
|---|---|---|---|---|---|
| A | 0.000 | 0.080 | 0.920 | 0.00 | 0.00 |
| B (×9) | 0.000 | 0.100 | 0.900 | 0.00 | 0.02–0.06 |
| C (×9) | 0.000 | 0.240–0.360 | 0.640–0.760 | 1.54–1.98 | 0.02–0.08 |

Uniform across all five families; not driven by any one of them.

## 4. What G2 did establish

1. **Constructed path invalidation deadlocks strict one-shot A\*–SFM.** Arm A
   deadlocks in **46/50** calibration scenarios (median 5794 steps). The
   retired G suite produced **zero** deadlocks in 75 scenarios. Invalidating
   the route by construction does what harder geometry could not, and this
   instantiates reviewer 1.3's premise.
2. **Waypoint advancement alone is inert here.** B vs A: advancement fires
   0.02–0.06 times per scenario and moves exactly one scenario out of deadlock
   (Δdeadlock −0.020, 1 discordant pair).
3. **Replanning converts deadlock into contact, not into safe arrival.**
   C vs B: Δdeadlock −0.14 to −0.26, Δunsafe +0.14 to +0.26, Δsafe **+0.000**,
   7–13 discordant pairs. Every run replanning rescues from deadlock arrives
   with contact.

## 4a. Status of this result relative to reviewer 1.3

G2 **provides a targeted negative stress-test response to reviewer 1.3.** It
does not "answer" the review comment without qualification, because the formal
evaluation pool was never touched: every number above is **calibration-stage
evidence** from a 50-scenario pool.

Permitted summary:

> In a targeted path-invalidation challenge set, strict one-shot A\*-SFM
> deadlocked in 46/50 calibration scenarios. Waypoint advancement alone had
> negligible effect, while event-triggered replanning reduced deadlock but
> converted those cases into unsafe arrivals rather than safe completion.

## 4b. The TTC-disabled confound bounds every claim here

G2 deliberately disables TTC to isolate planner recovery. Therefore it is
**not possible to attribute** the 0% safe completion to any one of:

- the recovery logic itself;
- the absence of predictive collision filtering in a 48-agent crowd;
- the interaction of the two.

**Forbidden claims — none is supported by this experiment:**

- "Replanning fails."
- "Replanning worsens safety."
- "Recovery cannot resolve route invalidation."
- "The full proposed architecture cannot safely recover from path invalidation."

**The permitted claim, and the only one:**

> Under the TTC-disabled G2 challenge conditions, replanning exchanged
> immobilization for contact without producing safe completion.

## 5. Decision

**The 150-scenario evaluation pool is NOT launched.** It remains untouched and
geometrically validated. This follows the frozen protocol and the Amendment F
precedent (negative calibration result, empty feasible candidate set).

**Forbidden responses, all of which would rebuild the error that retired G:**

- relaxing or re-specifying the +0.05 safety screen after seeing it reject;
- regenerating or re-tuning the pool until safe completions appear;
- softening the "safe" definition to admit contact;
- selecting a configuration from the descriptive table in §5' of the output;
- **creating an Amendment G3**, with TTC enabled or otherwise. G and G2 have
  already established that the planner concern is real and that a simple
  recovery layer does not obviously solve it. Reopening the experimental tree
  has lower marginal value for this resubmission than finishing the remaining
  reviewer-required work.

**Permitted wording:**

> Under deliberately constructed static path invalidation, strict one-shot
> A\*–SFM deadlocked in 92% of calibration scenarios. Adding waypoint
> advancement changed almost nothing. Adding replanning substantially reduced
> deadlock but produced no safe goal completions: recovered trajectories
> arrived with contact. On this pool the recovery layer trades immobilisation
> for collision rather than for safe arrival.

**Forbidden:** any claim that route recovery improves safe completion, and any
presentation of the calibration screen as evidence about arm C's safety.

## 6. Why no arm completes safely — hypothesis, not a finding, and not to be tested here

Not established by this run; see §4b for the claims this rules out. The pool may
be discriminating *deadlock vs contact* rather than *recovery success*.

A TTC-enabled variant would be scientifically interesting and is **deliberately
not being run.** G2 is closed and preserved as a negative targeted challenge
result. It is neither evaluated nor redesigned.
