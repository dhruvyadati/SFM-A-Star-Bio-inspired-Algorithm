# Amendment G — RETIRED 6 August 2026 (construct-validity failure)

> The original blockage suite failed its construct-validity check. Strict
> one-shot A*--SFM produced no deadlocks in 75 calibration scenarios and
> instead reached the goal unsafely in 57 cases. Because the suite did not
> create a meaningful route-recovery population, no three-arm comparison or
> evaluation-set analysis was performed.

**This is NOT the answer to reviewer 1.3.** The following statement is
accurate:

> Even in deliberately constructed blockage geometry, strict one-shot
> A*--SFM did not deadlock.

These are **overreach and are forbidden**:

> Therefore route recovery is unnecessary.
> This answers the replanning concern negatively.

The suite never instantiated the intended estimand. What it shows is that this
controller's failure mode is *contact rather than immobilisation* --- useful,
but not a comparison of one-shot planning against advancement and replanning.

**The 200-scenario evaluation pool is RETIRED UNEXAMINED**, not repurposed. It
belongs to a design that failed calibration validity, and reusing it would
carry that design's assumptions into a new experiment.

**Options considered and rejected.** Retargeting G around unsafe arrival would
be choosing a different experiment after seeing calibration data. Enabling TTC
to manufacture deadlocks would confound planner recovery with filter-induced
immobility and would reverse a correctly frozen decision purely because it
generates events.

The calibration pool did exactly what it exists for: it stopped an invalid
experiment before it reached the evaluation set.

**Superseded by Amendment G2** (path invalidation). What follows is the retired
design, retained for the audit trail.

---

# Amendment G — retired design (for audit)


## BLOCKER — the challenge suite does not produce blockage (6 Aug 2026)

Arm A run on all 75 calibration scenarios, before any calibration:

| Family | safe | unsafe | deadlock | other |
|---|---|---|---|---|
| narrow_passage | 8 | 7 | 0 | 0 |
| temporary_obstruction | 3 | 12 | 0 | 0 |
| concave_pocket | 2 | 13 | 0 | 0 |
| displaced_waypoint | 4 | 11 | 0 | 0 |
| blocked_waypoint | 1 | 14 | 0 | 0 |
| **TOTAL** | **18** | **57** | **0** | **0** |

**Zero blockage-type failures.** The suite stresses the controller in the
SAFETY dimension --- 57/75 unsafe arrivals --- but not in the MOBILITY
dimension it was built for. G's primary outcome is safe goal completion and its
motivating failure is blockage; with no blockage, recovery has nothing to
recover from and the three arms cannot be meaningfully compared.

This is consistent with the confirmatory study, where guided SFM deadlocked in
only 2.0% of scenarios without TTC. Guided SFM is simply reluctant to deadlock:
goal attraction plus waypoint guidance pushes it through gaps and around
pockets, at the cost of contact.

### Why this is not fixable by adjusting the geometry

The obvious response is to narrow the gaps and deepen the pockets until
deadlocks appear. **That would be tuning the benchmark until it produces the
phenomenon under study**, which is the same error as relaxing a feasibility
check to admit a family --- and it would make any subsequent recovery result an
artefact of how hard the geometry was pushed.

### Options, none yet chosen

1. **Report the null as the finding.** Even in deliberately constructed
   blockage geometry, strict one-shot A*--SFM does not deadlock; it arrives
   unsafely instead. Reviewer 1.3's premise --- that one-shot planning strands
   the robot --- is then answered with evidence, negatively.
2. **Re-target G's outcome** to what the suite does stress: displacement from
   the planned route, waypoint inaccessibility, and unsafe arrival. This
   changes the primary outcome after seeing arm A data, and would have to be
   declared as such.
3. **Run G with TTC enabled**, where deadlock is 14%. Previously rejected
   because it confounds route recovery with filter-induced deadlock, and that
   objection still stands.

Option 1 costs nothing further and answers the reviewer honestly. Option 2 is
defensible only if the redefinition is declared post-hoc. Option 3 was rejected
on frozen grounds and should not be revived merely because it would produce
events.

## Decisions closed

**TTC setting: DISABLED.** Reviewer 1.3 asks about the one-shot planner, not
the filter. Running G with TTC enabled would make the result conditional on a
safety layer that itself creates deadlock, and we could not tell whether route
recovery solves blockage or merely compensates for braking. The low 2% deadlock
rate under TTC-disabled is not a reason to enable it --- it is evidence that the
ordinary four-condition benchmark is the wrong population for G, which is why a
blockage-enriched suite is built instead.

**Scenario suite: hand-authored challenge families**, not random generation.
Permitted because the geometry rules are written before any scenario exists,
feasibility checks are defined, all three arms receive identical scenarios, and
the suite is described throughout as a *targeted challenge set, not
representative deployment sampling*.

**The 5a constants are NOT inherited.** The 0.05 m / 3 s trigger and two-attempt
cap were never implemented, never validated and never tuned. They are treated
as a candidate architecture requiring calibration on a disjoint pool, exactly
as Amendment F's parameters were --- and Amendment F showed that two of five
such parameters turned out non-discriminating, so assuming these are sensible
would repeat that error.

## Sample sizes and pools — frozen

| Pool | Per family | Total | Purpose |
|---|---|---|---|
| **calibration** | 15 | 75 | feasibility and coarse threshold screening ONLY |
| **evaluation** | 40 | 200 | the frozen three-arm comparison |

Maximum generation attempts: **400 per family per pool**. If a family cannot
reach its quota it is reported as under-filled rather than having its geometry
rules relaxed. Rejections are counted by reason.

The two pools use disjoint seed families, both disjoint from confirmation,
development and the Amendment F calibration set.

**ONE shared parameter set across all five families.** Optimising per family
would yield five family-specific controllers and would not support a general
recovery claim. Calibration selects a single set; evaluation applies it
unchanged.

## Arms — B is contained in C exactly

| Arm | Behaviour |
|---|---|
| **A** strict one-shot | one A\* path, sequential waypoint progression, no advancement, no replanning |
| **B** safe advancement | A, plus the advancement rule below. **No global replanning.** |
| **C** advancement + replanning | **exactly B**, plus replanning as the sole added component |

C contains B's advancement logic unchanged. If the two differed in any other
respect the B--C contrast would not isolate replanning.

## Safe advancement — geometric, auditable, no "skip when stuck"

Advancement fires only when ALL of the following hold:

1. the active waypoint has failed a progress test for a continuous
   `G_STALL` seconds (arc-length progress below `G_PROG` metres);
2. some later waypoint is line-of-sight reachable from the current position;
3. the connecting segment maintains at least `G_CLEAR` metres of clearance from
   every footprint-inflated obstacle along its whole length;
4. the target is the **earliest** feasible later waypoint, not the furthest.

Rule 4 departs from the §5a sketch, which proposed the furthest visible
waypoint. The earliest feasible target is the **minimal intervention** that
resolves the blockage, and §5a's own text warns that skipping can "aim the
robot at a later waypoint through an obstacle or cut an unsafe corner". It is
also consistent with Amendment F's frozen tie-breaking toward weaker
intervention.

Advancement may skip at most `G_MAXSKIP` waypoints in one event, and may never
select a target whose connecting segment crosses an obstacle boundary.

## Replanning — arm C only

Fires only when advancement is eligible by rule 1 but **no** later waypoint
satisfies rules 2--4. A fresh A\* is run from the current position. Subject to
a cooldown of `G_COOLDOWN` seconds and a maximum of `G_MAXPLAN` replans per
trial; after the cap, arm C behaves as arm B.

## Threshold sources — calibration, not inheritance

| Constant | Source |
|---|---|
| `G_STALL`, `G_PROG` | coarse screen on the 75-scenario calibration pool |
| `G_CLEAR` | **engineering rationale, not calibrated**: robot radius 0.35 m plus the 0.15 m TTC clearance already used elsewhere = **0.50 m** |
| `G_MAXSKIP` | **fixed at 3** — bounds corner-cutting without calibration |
| `G_COOLDOWN` | **fixed at 3.0 s** — anti-chatter, structural |
| `G_MAXPLAN` | **fixed at 2** — matches the §5a attempt cap, adopted as an engineering choice and labelled as such, not inherited as validated |

Only two constants are calibrated. The rest are fixed by rationale and recorded
as such, because Amendment F showed that assuming a parameter is sensible is
how non-discriminating parameters enter a study.

## Outcomes — primary is SAFE GOAL COMPLETION

Not deadlock reduction. A recovery method can cut deadlock by taking unsafe
shortcuts or by replanning excessively, and a deadlock-only primary would
reward exactly that.

**Primary:** safe goal completion, arm A vs arm C, paired within scenario.

**Reported separately, always:** deadlock; unsafe arrival/contact; replanning
count; waypoint-advancement count; path length; traversal time; computation
time.

**Secondary, ordered:** A vs B (isolates advancement); B vs C (isolates
replanning); path-length and displacement cost; cases where recovery fired and
still failed.

## Reporting — pooled AND per family

Both are always reported. Pooled answers whether recovery helps on the
challenge suite; per-family shows whether one constructed family drives the
conclusion. **The method is never selected on its best family.**

## Temporary obstruction must be controller-independent

Its dynamics are fixed by the scenario seed and cannot depend on robot
behaviour, or the family becomes a hand-tuned rescue case.

- **onset**: obstruction present from $t=0$
- **duration**: blocking pedestrians hold station for a seed-determined
  $T_\mathrm{block} \in [8, 20]$ s, then resume their goals
- **release**: time-triggered only; never conditioned on robot position, speed
  or state
- **observation**: the robot observes pedestrians as ordinary dynamic agents;
  the obstruction is not encoded in the static map
- **replanning input**: arm C replans against the **static** map only, so
  released obstruction does not silently feed the planner

## Arm B — advancement rule, fully specified

| Item | Rule |
|---|---|
| failure to progress | remaining arc length along the **current** path decreases by less than `G_PROG` over a continuous `G_STALL` seconds (both calibrated) |
| eligible targets | waypoints strictly after the active index, at most `G_MAXSKIP` = 3 ahead |
| line of sight | the straight segment from the robot to the candidate intersects no obstacle |
| clearance | that segment stays at least `G_CLEAR` = 0.50 m from every obstacle along its whole length |
| choice | the **earliest** feasible candidate, never the furthest |
| repeat | permitted, subject to cooldown |
| cooldown | `G_COOLDOWN` = 3.0 s after any advancement before another may fire |
| cap | `G_MAXADV` = 3 advancements per trial; afterwards arm B behaves as arm A |
| failure | if no candidate is feasible, nothing happens (arm B has no fallback) |

## Arm C — replanning rule, fully specified

| Item | Rule |
|---|---|
| order | **advancement is attempted first**. Replanning fires only when the progress test is satisfied AND no candidate passes B's line-of-sight and clearance tests. |
| trigger | as above |
| start state | the robot's current position, snapped to the nearest unblocked grid node |
| map | **static geometry only** — held pedestrians are never planner obstacles |
| active waypoint | reset to index 0 of the new path |
| no path found | keep the existing path, count a failed replan, continue as arm B |
| cooldown | `G_COOLDOWN` = 3.0 s |
| cap | `G_MAXPLAN` = 2 replans per trial; afterwards arm C behaves as arm B |

`G_MAXADV` = 3 is fixed by engineering rationale, not calibrated: beyond three
advancements the original route is likely wrong, which is a planning failure
rather than a blockage. Recorded as a choice, in the same way `G_MAXPLAN` = 2
is adopted from the 5a sketch as an engineering choice rather than inherited
as validated.

## What arm C's replanner perceives — frozen, with consequences stated

**Arm C replans against the STATIC map only.** Held pedestrians are never
converted into planner obstacles. They remain pedestrians for the local
controller throughout.

This was already implied by "arm C replans against the static map only", but
the consequence was not spelled out, and it materially changes what each
family tests:

| Family | What arm C can actually do |
|---|---|
| `narrow_passage` | **stress test for unnecessary or destabilising recovery activation on a geometrically constrained but valid route.** Replanning cannot widen a gap, and the correct behaviour is usually to do nothing. |
| `temporary_obstruction` | the planner cannot route around the blockers. Success comes from **waiting or advancement**, NOT from pedestrian-aware replanning. The blockers clear on their own timer. |
| `concave_pocket` | replanning from inside the pocket **can** help: the static geometry is known and a new path from the current position escapes it |
| `displaced_waypoint` | replanning **is** the intended mechanism: the robot is off-route, and a fresh plan from its actual position is genuinely better |
| `blocked_waypoint` | blockers hold long enough that waiting alone should fail, so **advancement** is the operative mechanism |

So arm C is a *static-geometry* replanner. The manuscript must say so: it
answers "can the planner recover when the robot is displaced or trapped by
static geometry?", not "can the planner route around people?". A
pedestrian-aware replanner is a different system and is not tested here.

All five families, with their intended failure mode fixed BEFORE calibration:

| Family | Tests |
|---|---|
| `concave_pocket` | static-geometry replanning |
| `displaced_waypoint` | static-geometry replanning |
| `temporary_obstruction` | waiting / advancement under dynamic pedestrian blockage |
| `blocked_waypoint` | waiting / advancement under dynamic pedestrian blockage |
| `narrow_passage` | that recovery does **not** destabilise an already-constrained but valid route |

Reported per family, so a pooled result cannot hide which mechanism did the
work --- and so `narrow_passage` acts as a check against recovery firing where
it should not.

**Forbidden claim:** that arm C "handles dynamic obstacles". It does not
replan around people.

## Geometry rules, fixed before any scenario is authored

Each family below is parameterised, and seeds vary pedestrian placement and
timing within a family. Every scenario must satisfy the feasibility checks in
the following section or be rejected and counted.

| Family | Geometry rule |
|---|---|
| `narrow_passage` | two rectangular walls leaving a gap of width $w \in [1.2, 2.0]$ m on the straight line from start to goal |
| `temporary_obstruction` | a narrow passage whose gap is occupied by pedestrians whose goals lie inside it, so the passage clears only after they pass |
| `concave_pocket` | a U-shaped obstacle opening toward the start, positioned so the straight-line path enters the pocket |
| `displaced_waypoint` | a wall placed so that crowd pressure pushes the robot to the far side, leaving the active waypoint behind an obstacle |
| `blocked_waypoint` | pedestrians with stationary goals placed on the active waypoint, while a later waypoint retains clear line of sight |

## Feasibility checks, fixed before authoring

1. A feasible A\* path from start to goal must exist on the inflated grid.
2. No obstacle may overlap another obstacle.
3. The robot start and goal must be outside every obstacle by at least one
   robot radius.
4. No pedestrian may start inside an obstacle.
5. Rejections are counted and reported by family.



**A three-arm ablation of global-route recovery, answering reviewer 1.3.**

> Does event-triggered route recovery reduce blockage relative to the strict
> one-shot planner, and what safety or path-cost consequences does it introduce?

Separate from Amendment F by design. F modifies the **TTC safety layer**; G
modifies **global-route recovery**. They are not crossed, and no interaction
between them is claimed or estimated.

## Why this exists

Reviewer 1.3 (Reviewers 1 and 4) challenges the one-shot planner directly:
prolonged blockage, inaccessible waypoints, large displacement from the original
route. PROTOCOL §5a specified a fallback for exactly this but **was never
implemented** (audit, 3 Aug 2026). Amendment F does not answer 1.3 — it
releases the brake, not the route. Leaving 1.3 wholly unanswered is an
avoidable resubmission risk. Also partially addresses 1.37.

## Arms — three, differing only in route recovery

| Arm | Behaviour |
|---|---|
| **A. strict one-shot** | current system: one A\* path, sequential waypoint progression, no recovery |
| **B. waypoint advancement** | on lack of progress, advance to the furthest later waypoint with verified collision-free line of sight from the current position, tested against footprint-inflated obstacles |
| **C. advancement + replanning** | as B; if no later waypoint is safely visible, run a fresh A\* from the current position |

**The local controller is identical in all three arms** — same SFM parameters,
same integrator, same TTC setting held fixed. G does **not** cross with
TTC-recovery from F.

**TTC setting: RESOLVED as disabled** — see "Decisions closed" above.

## Do NOT resurrect the §5a constants

§5a proposed a 0.05 m / 3 s progress trigger and a two-attempt cap. **Those
numbers were never validated, never implemented, and never tuned on anything.**
They are a *candidate architecture*, not frozen values. Each must be calibrated
on a disjoint set or justified on engineering grounds before evaluation:

| Constant | §5a candidate | Status |
|---|---|---|
| progress threshold | 0.05 m | **must be calibrated or justified** |
| progress window | 3 s | **must be calibrated or justified** |
| recovery attempt cap | 2 | **must be calibrated or justified** |
| line-of-sight inflation | "footprint-inflated" | **must be made numerically explicit** |

Reusing them unexamined because they appear in a document labelled FROZEN would
repeat the error the audit just exposed.

## Outcomes

**Primary:** blockage/deadlock rate, A vs C, paired within scenario.

**Secondary, ordered:**
1. blockage rate, A vs B — isolates advancement from replanning
2. unsafe-arrival rate across arms — does recovery buy mobility with safety?
3. path length and goal time — the displacement cost reviewer 1.3 raises
4. recovery trigger counts and attempt distribution (a §5a reported metric)
5. cases where recovery fired and still failed

**Honesty requirement, carried over from §5a verbatim in intent:** arms B and C
change "one-shot A\*" into *one-shot under normal operation with event-triggered
recovery*. The manuscript must say so plainly. Claiming "no replanning" while
quietly adding replanning would be far more damaging than the honest
description — and the mirror error, claiming recovery that does not exist, is
what the audit found.

## Scenarios — RESOLVED

Superseded by the frozen design above: a hand-authored blockage suite with
geometry rules and feasibility checks fixed before authoring, in disjoint
calibration (75) and evaluation (200) pools. The earlier text weighing "new
seeds of existing conditions" against "extend the generator" is withdrawn; the
existing four benchmark conditions contain no blockage geometry and are the
wrong population for this question.

## Status

**Exploratory, reviewer-facing.** Not confirmatory, not powered as such, and it
must not be presented as one. It answers a specific reviewer challenge about an
architectural limitation.
