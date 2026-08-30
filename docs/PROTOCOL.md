# Frozen Experimental Protocol — v2

**Status: DRAFT. Freeze before Stage 2 begins. Do not start the full matrix until this is signed off by all authors.**

Every ambiguity resolved here is one that would otherwise force a re-run of
the entire experimental matrix when noticed mid-campaign. Changes after the
freeze require a version bump and a documented reason.

| Field | Value |
|---|---|
| Protocol version | 2.0-draft |
| Frozen on | *(pending)* |
| Reference implementation | `SFM-A-Star-Bio-inspired-Algorithm/v2/SfmParams.m` |

---

## 1. Workspace and geometry

| Item | Value | Note |
|---|---|---|
| Grid cell size | 1.0 m | Declared, not implied. Resolves comment 1.28 |
| Workspace | 50 × 50 m | `[1,50]²` |
| Grid resolution | 1 m, 8-connected | Diagonal edges weight √2 |
| Robot radius | 0.35 m | Disc model, replaces v1's point agent |
| Pedestrian radius | 0.30 m | |
| Graph safety margin | 1.0 m | > robot radius, so the pruned graph is conservative |
| Obstacle area units | m², shoelace, **pre-rasterisation** | Resolves comment 1.29 |
| Obstacle vertices | 3–10 default; 3–20 in the vertex sweep only | Two generators; must be stated. Comment 1.27 / 2.6 |
| Obstacle overlap | **REJECTED** in primary generated maps | See §1a |
| Static field cache | `MapField.m`, memoised by (seed, grid step); verified by `Test_MapFieldCache` | Depends only on map, goal, controller constants — never on outcomes |

### 1a. Map acceptance rules — FROZEN

Generated obstacles may **not** overlap each other, the start region, the goal
region, or the required clearance buffers. Maps with no feasible A* path on
the inflated grid are rejected.

Rationale: overlapping polygons make "obstacle number", "area", and "shape
complexity" uninterpretable, because two nominal obstacles may behave as one
composite. Every sweep in the paper varies exactly those quantities.

Overlapping obstacles may return later as a distinct **clustered-obstacle
scenario**, but never in the primary random-generation protocol.

Every rejection must be reported:

- number and percentage of rejected maps;
- rejection reason (overlap / no feasible path / vertex or area constraint);
- number of generation attempts;
- confirmation that the same accepted maps were used for **every** method.

### 2. Dynamics

| Item | Value |
|---|---|
| Mass | Unit mass; all force terms are accelerations in m/s² |
| Integrator | Semi-implicit (symplectic) Euler: **v first, then x with updated v** |
| Timestep | 0.01 s |
| Relaxation time τ | 0.5 s |
| Preferred speed | 1.34 m/s pedestrian, 1.30 m/s robot |
| Speed cap `vMax` | 2.0 m/s |
| Minimum speed | **None.** v1's `vMin = 1` is removed |
| Goal deceleration | Desired speed tapers linearly to 0 within `dSlow` = 2 m |
| Agent repulsion | `A·exp((rᵢ+rⱼ−d)/B)·n̂`, A = 2.1 m/s², B = 0.30 m, cutoff 5 m |
| Obstacle repulsion | `A·exp((rᵢ−d)/B)·n̂`, A = 10 m/s², B = 0.20 m, cutoff 3 m |
| Contact stiffness | 200 s⁻², applied on physical overlap only |

All repulsion uses **unit direction vectors** and decays monotonically.
Interior penetration is corrected outward.

## 3. Trial structure

| Item | Value |
|---|---|
| Start / goal | (1,1) → (50,50), fixed |
| Pedestrian count | 48 default; swept 0–96 in the density study |
| Pedestrian init | Uniform over free space; velocity ~ N(0,1) per component |
| Pedestrian goals | Uniform over free space, **resampled on arrival** within 1.0 m |
| Arm isolation | Each method runs in its **own simulation**. Arms never interact |
| Pairing | Both arms consume the same scenario seed → identical obstacles, identical crowd initial conditions, identical goal-resampling stream |
| Horizon | 15 000 steps = 150 s ≈ 2.8× free-space traverse time |

## 4. Outcome taxonomy

Exactly one outcome per trial, evaluated in this order:

| Outcome | Criterion |
|---|---|
| `infeasible` | No A* path exists on the pruned graph. **Counted and reported**, excluded from both arms |
| `success` | Within `rGoal` = **0.5 m**, speed ≤ **0.1 m/s**, held **1.0 s** (100 steps), **and no contact during the dwell**. Sensitivity reported at 0.25 / 0.5 / 1.0 m |
| `deadlock` | Speed < 0.05 m/s for 500 consecutive steps (5 s) while outside `rGoal`. **Onset** = first step of that window, which is what the horizon task predicts |
| `timeout` | Horizon exhausted without success or deadlock |

A single instantaneous entry into the goal region does **not** count. The
2 m latched criterion was replaced because it was too coarse to defend against
a 0.35 m robot on a 1 m grid, and it made the final-distance metric almost
tautological — every successful run ended within 2 m by construction.

## 5. Safety and social definitions

All thresholds are **surface clearance**, not centre distance:

$$c_{rh}(t) = \lVert p_r(t) - p_h(t)\rVert - r_r - r_h$$

| Item | Definition |
|---|---|
| Collision | `c ≤ 0` |
| Near-collision | `0 < c < 0.10 m` — a remaining surface gap, **not** centre distance |
| Collision count | Contact **episodes**, not per-step samples |
| Continuous checking | **Swept-circle** between successive positions. Discrete per-step sampling can tunnel through a contact at 2 m/s and 0.01 s steps |
| Minimum clearance | Reported continuously over the trajectory, interpolated between steps |
| Intimate-zone intrusion | Centre distance < 0.45 m (Hall) |
| Personal-zone intrusion | Centre distance < 1.20 m (Hall) |
| Smoothness | **RMS** ‖da/dt‖ is primary (see §6). Mean ‖a‖ and mean ‖da/dt‖ secondary. Acceleration measured AFTER the speed clamp and safety filter, i.e. realised not commanded |

### 5a. Blockage handling — SPECIFIED BUT NEVER IMPLEMENTED

> **PROTOCOL DEVIATION, recorded 3 August 2026.** Section 5a specified a
> blockage-recovery mechanism, but a code audit established that it was never
> implemented in either evaluation engine. All reported experiments therefore
> used strict one-shot A\* guidance with ordinary sequential waypoint
> progression and no event-triggered waypoint skipping or replanning. The
> discrepancy was identified after the completed studies and is reported as a
> protocol deviation. The §5a three-way ablation remains unexecuted.
>
> Precisely what this does and does not mean:
> - the written protocol included an intended feature that was never built;
> - the actual experimental system was **consistent across all arms**;
> - all results describe the strict one-shot implementation and remain valid
>   for it;
> - the manuscript and response letter must follow **the code actually run**,
>   not this section.
>
> What the engines do instead: waypoint progression when within `rWaypoint`
> = 2.0 m (normal sequential advance, not progress-triggered, cannot skip to a
> later visible waypoint); deadlock declared after 5.0 s continuously below
> `stallSpeed` = 0.05 m/s outside the goal radius. There is no `blocked`
> outcome category.
>
> The text below is retained verbatim as the original specification. It
> describes intended, unexecuted behaviour.

#### Original specification (unexecuted)

Unconditional waypoint skipping is **rejected** as the primary fallback: a
skipped waypoint can aim the robot at a later waypoint through an obstacle or
cut an unsafe corner. The frozen behaviour is:

1. The robot may stop (`vMin = 0`).
2. Declare lack of progress when the distance to the active waypoint improves
   by less than **0.05 m** over a continuous **3 s** window.
3. Advance to the **furthest later waypoint with verified collision-free line
   of sight** from the current position, tested against footprint-inflated
   obstacles.
4. If no later waypoint is safely visible, run a **fresh A\* from the current
   position**.
5. After **2** recovery attempts, classify the trial as `blocked`.

Recovery trigger counts are a reported metric. These constants are tuned on
**development seeds only**, then frozen before evaluation.

**Honesty requirement.** This changes "one-shot A\*" into *one-shot under
normal operation with event-triggered recovery*. The manuscript must say so
plainly, and report the three-way ablation:

- strict one-shot, no recovery;
- safe waypoint advancement only;
- event-triggered A\* recovery.

Claiming "no replanning" while quietly adding replanning would be far more
damaging than the honest description.

## 6. Statistical analysis plan

**Pre-designate before running the matrix. Do not choose endpoints after seeing results.**

| Item | Choice |
|---|---|
| **Primary endpoint** | **SAFE SUCCESS** — reached the goal *and* zero contacts. Goal success alone is reported only as a secondary measure, because it overstates performance badly (100% goal vs 29% safe in the first batch) |
| Primary comparison | Paired, exact McNemar on shared seeds |
| Primary effect size | Paired success-rate difference in percentage points, with CI; matched odds ratio reported alongside, with explicit handling of a zero discordant cell |
| Co-primary (safety) | Contact **duration** and **maximum penetration**, paired Wilcoxon signed-rank. Episode counts are reported but are NOT primary: `gapPed` is a min over all pedestrians, so consecutive contacts with different people merge into one episode while bounce-chatter splits one contact into several |
| Smoothness | **RMS** jerk over the trajectory, m/s³, defined explicitly. Mean \|da/dt\| is reported alongside but is dominated by rare contact spikes |
| Proximity | Reported as a **fraction of trial duration**, not total seconds, so unequal trial lengths cannot drive the comparison |
| Effect-size naming | Every effect size named explicitly (rank-biserial, Cliff's δ, Cohen's h). Never reported as a bare "effect" |
| Path length | Reported over **all** trials as a penalised metric, plus final distance-to-goal. Successful-runs-only figures are secondary and must carry an explicit selection-bias caveat (comment 1.23) |
| Proportion CIs | **Wilson score** |
| Mean CIs | **Percentile bootstrap**, 10 000 resamples |
| Multiple comparisons | **Holm–Bonferroni** across swept conditions within each family |
| Failure handling | Failures are never dropped from success/collision analyses |

Implemented toolbox-free in `v2/StatsLib.m`.

## 7. Baseline fairness contract

Every baseline must be run under identical conditions. Any deviation must be
stated in the manuscript.

| Requirement | Rule |
|---|---|
| Map and crowd | Identical scenario seed |
| Robot geometry | Same radius, same `vMax` |
| Success / collision criteria | Identical |
| Map knowledge | State explicitly whether the baseline gets the static map |
| Control frequency | State per method; equalise where the method permits |
| Tuning budget | Equal number of tuning trials per method, on a **held-out seed set** disjoint from the evaluation set |
| Implementation | Record library, version, commit |

**ORCA caveat.** ORCA assumes *reciprocal* avoidance — every agent runs ORCA.
Our pedestrians are SFM agents, so that assumption is violated. This must be
stated as a limitation rather than glossed; the fair framing is "ORCA robot in
an SFM crowd," not "ORCA vs SFM."

## 8. Reproducibility

| Item | Rule |
|---|---|
| Seeding | One master seed derives all per-trial seeds; every seed logged to `runs/` |
| Determinism | Scenario fully determined by seed; any trial replayable via `MakeScenario(seed, P)` |
| Pedestrian goal stream | Separate `RandStream` so arms stay matched regardless of run length |
| Toolbox dependencies | **None beyond base MATLAB + Mapping Toolbox.** v1's dependence on the paid Statistics Toolbox broke reproducibility |
| Release | Code, protocol, seed logs, and raw per-trial CSVs |

---

## 9. Primary configuration — selection rule, FROZEN

The primary condition is **not** chosen by picking a number now. It is
calibrated by a written rule, on seeds excluded from all reported results:

1. Use a **calibration seed set disjoint** from the evaluation set.
2. Vary obstacle count, density, crowd speed, and geometry.
3. Select the configuration where the corrected hybrid reaches **≈70–90%
   success** (target ~80%) while the principal baseline stays meaningfully
   lower.
4. Freeze it **before** any method comparison.
5. Retain easy / moderate / severe conditions as secondary strata.

Rationale: at 10 obstacles the hybrid succeeds on every trial, so ablations,
sensitivity sweeps, and parameter studies would all return flat lines. ~80%
leaves enough successes for efficiency and smoothness analysis, enough
failures for deadlock and trap validation, and headroom for a stronger method
to show improvement.

The criterion is written down first so the configuration cannot be chosen for
producing a favourable result.

## 10a. Horizon-based deadlock prediction — FROZEN

Frozen BEFORE the large batch is generated, so the dataset is not built around
a labelling scheme already known to be the wrong question. Endpoint
association ("is the final stall position in a flagged region") is replaced by
prospective prediction.

**Task.** For an eligible trajectory state, will a deadlock BEGIN within the
next H seconds?

| Item | Value |
|---|---|
| Horizons | H ∈ {3, 5, 10} s. **Primary = 5 s**; 3 and 10 are sensitivity analyses, all three prespecified |
| Deadlock onset | First step of the stall window, i.e. `terminationStep − stallSteps + 1`. **Not** the termination step — using that would shift every label by 5 s and make the task easier than stated |
| Eligible states | Sampled every 0.5 s (50 steps), after the robot has first reached 0.5·v₀, and ≥3 m from both start and goal |
| Positive | Deadlock onset occurs within (t, t+H] |
| Negative | No deadlock onset within (t, t+H] |
| Exclusion | States within H of trial termination in non-deadlock trials — the label is unknowable there |
| Samples per trial | ONE positive and ONE hard negative per trial by default. Adjacent trajectory states are strongly correlated, so pooling hundreds of negatives per trial against one positive would shrink intervals artificially |
| Negative matching | Chosen to resemble the positive on elapsed time, distance to goal, obstacle clearance, local pedestrian density, and speed — so trajectory PHASE cannot do the work |
| Split unit | **Whole map.** No state from one map may appear in both calibration and test |
| Root seeds | Equilibrium candidates seeded from the map field only, never initialised from an observed deadlock position |

**Reported metrics.** AUC alone is insufficient — a diagnostic that alerts
constantly can have high AUC and no practical value. Also report:

- precision–recall (positives are rare among eligible states);
- sensitivity at fixed flagged-area and fixed false-positive levels;
- calibration;
- **lead time** before deadlock onset;
- trial-level detection rate;
- **false alerts per trajectory and per minute.**

**Prespecified multiplicity family.** The scientific comparison family is
exactly three predictors: `staticForce`, `staticAttract`, `obsClearance`.
Holm–Bonferroni is applied across those three. `randomScore` and
`distFromStart` are **pipeline controls, not hypotheses**, and are excluded
from the family — `randomScore` must land near AUC 0.5 or the evaluation is
broken, and `distFromStart` must be weak or phase leakage remains. This family
is fixed here, before the confirmatory data exists.

**Failure-class scoping.** Deadlocks are classified as static-equilibrium,
pedestrian-blockage, controller-induced (filter waiting), or other. The static
field represents static structure only, so it is expected to predict the first
class. Claims are scoped to the classes it actually predicts.

## 10. Trap-diagnostic validation — FROZEN

| Item | Rule |
|---|---|
| Curve | **Deadlock-capture vs flagged-area**. Not called ROC — the x-axis is a workspace-area fraction, not a false-positive rate over labelled negatives |
| Summary | Area under the capture curve; reported as such, never as "AUC" |
| Chance baseline | Expected capture under random flagging = flagged-area fraction |
| Headline statistic | **Lift** = capture rate ÷ flagged-area fraction |
| δ selection | **Held out.** Calibration half selects δ; evaluation half reports it |
| Equilibrium test | **Full 4×4 Jacobian** `[0 I; A −I/τ]`. The 2×2 position Jacobian is necessary but not sufficient and produces false positives on underdamped spirals — verified against simulation in `Test_StabilityCriterion.m` |
| Terminology | "Locally attracting equilibrium **candidates** of the frozen instantaneous field." Not a stability proof for the time-varying system |

---

## Remaining open items

1. **SocNavBench feasibility** — two-day spike, must run before Stage 2 is
   designed around it. Fallback if it fails: **HuNavSim** (ROS 2-native,
   human-aware navigation with social metrics built in) in preference to
   plain Gazebo, or driving the native engine with ETH/UCY trajectory data.
2. **Development vs evaluation seed sets** — must be declared and separated
   before any tuning begins.
