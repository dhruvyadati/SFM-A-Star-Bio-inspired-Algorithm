# Paper Log

Running record of what must reach the manuscript. Everything here is either a
result, a retraction, or a decision with a reason. Updated as work proceeds.

**Status key:** `SOLID` reproduced and checked · `PILOT` encouraging, underpowered ·
`DEAD` did not survive · `OPEN` not yet tested

---

## 1. Claims from the submitted version that DID NOT survive

These must be removed or rewritten. Several are in the abstract.

| Claim | Status | What actually happened |
|---|---|---|
| "reducing the average path length by 36%" | **DEAD** | Paired on matched scenarios under corrected physics: 73.7 m vs 74.1 m, saving 1.2 m, p = 0.068. The original 105.9 vs 164.4 compared unpaired successful-runs-only samples with n=280 vs n=81, on a baseline handicapped by broken repulsion and an inability to stop. |
| "preserves global optimality" (line 149 of the .tex) | **DEAD** | A\* is optimal on the static grid graph only. Once SFM, braking, or waypoint advance modify the executed trajectory it is not globally optimal. Lines 140, 207, 370 (A\* on the static graph) are fine; 149 and 562 must go. |
| 3× smoother motion | **DEAD** | Artifact of using mean \|da/dt\|, which is dominated by rare contact spikes. Under RMS jerk the arms are indistinguishable. Also the original jerk metric used the *commanded force* rather than realised acceleration. |
| "immune", "unaffected", "decoupled" from obstacle complexity | **DEAD** | Success still degrades under harder conditions. |
| Implied collision-free navigation | **DEAD** | Both methods collide. Safe success (goal + zero contact) is 29% for the hybrid, 5% for SFM-only, against 100%/33% goal success. |
| "human-like" motion, social compliance | **DEAD** | No comfort or preference measurement was ever made. Personal-zone occupancy is an objective proximity statistic, not a comfort measure. |
| Trap ratio 0.20 as evidence of hazard | **DEAD as posed** | Unfalsifiable without checking whether failures land in flagged regions. Replaced by lift over chance. Also computed with softened parameters (a_s 3000, r_inf 3.5) that did not match the navigation runs (10000, 1.0). |
| A\* planning time as a computational-efficiency advantage | **REVISED** | The 21 s cost was graph construction, not search. Search is ~11 ms; graph build ~0.85 s after optimisation. Report them separately. |

## 2. Findings that survive

| Finding | Status | Numbers |
|---|---|---|
| Global waypoint guidance sharply improves goal completion | **SOLID** | 100% vs 33% goal success, n=100 matched. McNemar exact p = 1.4e-20, 67 discordant pairs vs 0. Wilson CIs [96.3, 100] and [24.6, 42.7]. |
| Guidance eliminates deadlock | **SOLID** | 0% vs 65% deadlock. |
| Distance-to-goal over ALL trials (no selection bias) | **SOLID** | 1.38 m vs 27.76 m, signed-rank p = 1.3e-13, rank-biserial −0.854. Note the hybrid figure is partly definitional given the goal radius. |
| Soft social repulsion does not guarantee collision avoidance | **SOLID** | Goal drive v0/τ = 2.60 m/s² exceeds social repulsion at contact (aPed = 2.10). Robot pushes through a pinned pedestrian and stays in contact for the whole traverse. Faithful to Helbing's formulation, not a bug. |
| Contact penetration is dynamic overshoot | **SOLID** | overlap ≈ v/√k; 92 mm predicted at 1.3 m/s, 111 mm observed. *Derivation assumes undamped unit-mass contact absorbing all kinetic energy — state this.* |
| Personal-zone occupancy differs | **SOLID** | 6.7% vs 23.4% of trial duration. Durations were near-identical (58.9 vs 57.7 s) so this is not a length artifact. Objective proximity only. |
| Guidance × safety-filter interaction | **PILOT** | Δ = +0.400, 95% CI [+0.100, +0.700], perm p = 0.033, **Holm-adjusted p = 0.098** across 3 contrasts. Simple effects +0.300 with guidance, −0.100 without. n=20 development. |
| Filtering without guidance is harmful | **PILOT** | `sfm-ttc`: 5% safe success, 85% deadlock, vs `sfm` 15%/50%. Mechanism visible in secondary metrics — braking raises personal-zone occupancy and exposure. |
| TTC has a low-relative-speed disengagement mode | **SOLID** | As \|u\| → 0, t\* → ∞ and the horizon test stops firing. Requires a current-separation floor. *Not novel — established in VO/CBF work. Frame as a property of this architecture, not a discovery.* |
| Static force field discriminates deadlock from near-stall-that-escaped | **PILOT** | With hard negatives (min-speed point of successful trials, gated past acceleration): `staticAttract` AUC 0.856, p = 0.0098, Holm-adj 0.039. Leakage controls clean: randomScore 0.600 (noise floor), distFromStart 0.522. n = 10 pos / 9 neg. |
| Static field does NOT prospectively predict deadlock at H=5s | **PILOT, NEGATIVE** | `staticAttract` AUC 0.546 vs `randomScore` control 0.538 — no signal. `staticForce` 0.439, below chance. Holm-adj p = 0.71 for both. **Caveat: test compromised** — positives sit 5 s before onset (early, median 22.8 s, 19.5 m from start) while negatives are min-speed points of successful runs (late, 29.6 s, 29.2 m). Cliff d ≈ 0.5 on three nuisance variables; only 42 negatives for 40 positives so time-matching had no slack. FIX: draw negatives from within deadlock trials (states >H before onset), which are phase-matched by construction. |
| ~~Stability refinement carries the result~~ | **RETRACTED** | The AUC 0.856 had no valid implementation behind it. The grid classifier applied the Jacobian test at points where ‖F‖ ≠ 0, which are not equilibria — the same category error as evaluating stability at a moving state, in position space. It also relied on `Re(λ) < 0` where the open-space goal field gives eig(A) = {0, −v₀/(τr)}: the exact ZERO eigenvalue is marginal, finite differences return ±ε, and noise fell negative ~90% of the time. Measured: **89–96% of free space flagged**, worsening with finer grids. Gating on ‖F‖ ≤ tol and adding a stability margin drops it to 0–2 cells of ~38,000, because equilibria are MEASURE-ZERO and a grid misses them. **No viable grid operating point exists.** |
| Trap ratio was never established as a basin measure | **SOLID (methodological)** | Equilibria are isolated state-space objects. Their BASINS can have positive area, so an equilibrium-related workspace fraction is not inherently impossible — but it cannot be obtained by counting grid cells passing a local force threshold or a noisy Jacobian test, which is what the submitted 0.20 did. Neither step established that a counted cell belonged to the basin of a verified attracting equilibrium. Estimating basin area requires verified roots plus a separate basin-membership procedure (e.g. backward integration), neither of which exists here. |
| The diagnostic is NOT a transformed obstacle-clearance map | **PILOT** | `obsClearance` AUC 0.478 — no discriminating power. Its earlier 0.889 was an artifact of the broken negatives. This was the main threat to the diagnostic's novelty. |
| Every SFM-only deadlock involves an obstacle | **SOLID** | 89 deadlocks across 7 conditions: 30% static, 70% mixed, **0% pure pedestrian**, 0% controller. No deadlock occurred with an obstacle gap > 1 m. Pedestrians alone do not trap the robot; they move on. Supports using a static map field as the instrument. |
| Pedestrian presence at onset is 1.65× enriched over an EMPIRICAL null | **PILOT** | Onset 0.667 vs matched non-deadlock states 0.403 within the same trials; paired signed-rank p = 0.018, rank-biserial 0.818, n = 12. **The earlier 5.5× figure was wrong** — it used a uniform-density Poisson null (0.127) that assumed independent uniformly distributed pedestrians. They are neither: pedestrians occupy the same navigable space as the robot, so the true baseline is 0.403. Corroborated by `fastPed` reducing deadlock (40% vs 50%), which incidental co-occurrence would not predict. |
| Velocity DOES affect capture outcome, via position displacement | **PILOT** | Sweep over 36 states across 3 maps, 4 directions x 5 speeds: median endpoint spread from velocity choice alone is 1.9 m, max 5.7 m at 5 s. Capture classification flips for 6% (2 s), 11% (3 s), 17% (5 s) of states. **RETRACTS an earlier claim that velocity is irrelevant beyond 1 s** — that rested on two velocities at ONE position. The e^(−t/τ) decay applies to the velocity TRACE, but velocity acts on outcome through position displacement during the transient (~1 m at v₀), and basin membership is positional. Indirect but durable. |
| Composite capture score is invalid below ~2 s horizon | **KNOWN FLAW** | Flip rate is 1.00 at H = 0.5 and 1.0 s because achievable progress over 0.5 s is 0.65 m, so the progress term cannot separate "trapped" from "little time elapsed" and initial speed dominates. Needs different normalisation before short-horizon results are interpretable. |
| 5 s deadlock timing not recoverable from the tested MAP-ONLY descriptors | **PILOT, NEGATIVE** | Scoped statement only. Does NOT establish impossibility, and does NOT apply to state-inclusive descriptors: an instantaneous state can carry pedestrian positions, velocities, local flow and filter state, none of which the tested field includes. `ForwardCapture` remains UNTESTED on the horizon task and is no longer demoted, since it captures the velocity-displacement effect the position-only score misses. |
| Static field is a ZERO-VELOCITY diagnostic | **SOLID** | The goal term omits −v/τ, so `F.stable` answers "is this position an attracting equilibrium for a robot AT REST". This explains why endpoint scoring works (robot is at rest at deadlock, assumption holds, AUC 0.856) and prospective scoring fails (robot moving at ~v₀ five seconds earlier, assumption void, AUC 0.546 vs random 0.538). The fix is NOT a Jacobian at the moving state — stability is defined only at an equilibrium, and eigenvalues at v≠0 say nothing about attraction. Momentum requires forward integration (`ForwardCapture`). **Terminology: never call a moving state "stable"; use capture susceptibility / forward trapping risk.** |
| Deadlock rate scales with obstacle area and crowd density | **SOLID** | bigObs 85%, highDens 90%, cluttered 70% vs baseline 50%; sparse and fastPed 40%. |
| SFM-only is unsafe in every condition tested | **SOLID** | Safe success 0–15% across all 7 conditions. |
| Robot social-repulsion gain contributes little | **SOLID** | +0.050 [0.000, +0.150], p = 1.0; −0.050 on top of TTC. Earlier 65% was a crowd-contamination artifact. |

---

---

## 8. Cross-language platform and ORCA baseline

### Validated infrastructure

| Component | Evidence |
|---|---|
| Python SFM engine | Trajectory agreement with MATLAB at **1e-14 m** across 6 canonical fixtures, against a 0.05 m prespecified tolerance. Identical outcomes, step counts, path lengths. |
| Vectorised obstacle forces | Behaviour-preserving; cross-validation still passes at roundoff. Pre-vectorisation engine frozen as `py/sfm_prevec_reference.py`. |
| RVO2 / ORCA | Reference implementation (sybrenstuvel/Python-RVO2 @ `c2c46ba`). 10 canonical checks pass. |
| A*-ORCA route following | Path lengths within 0.5% of A*-SFM on 4 shared scenarios; zero waypoint reversals; no corner cutting. |

Statement this supports: independently implemented engines produced trajectory
agreement at numerical roundoff across canonical cases, giving strong
end-to-end evidence that update order, integration, force evaluation,
clamping, goal taper, obstacle handling and deterministic goal-pool logic were
reproduced consistently. It is end-to-end evidence, not separate proof of each
detail.

**Runtime.** Python 2.46 ms/step vs MATLAB 2.7 ms/step — comparable
single-step runtime on the tested hardware. The ~14x batch advantage is an
**infrastructure and licensing** result (no Parallel Computing Toolbox
available), belongs in the reproducibility appendix, and is not a method
performance claim.

### ORCA baseline: three defects found before any comparative run

1. **`maxSpeed = 0` was inert, not merely distorting.** Intended as a
   full-responsibility mechanism, it produced results *bit-identical* to
   standard reciprocal ORCA, because externally driven pedestrians have their
   state overridden each step and RVO2 never consults their max speed. Any
   ORCA safety result predating this is void.
2. **Reciprocal ORCA is under-protective against non-yielding pedestrians.**
   Contact in 2 of 3 canonical cases (min separation 0.641 and 0.649 m against
   a 0.650 m contact distance). Confirmed real, not discretisation: swept
   separation equals sampled exactly, and refining dt 4x moves it only
   0.6412 -> 0.6477, converging toward the threshold from below.
3. **A robot-radius buffer leaks into obstacle conservatism.** Inflating the
   robot radius shifts static-obstacle standoff by exactly the buffer
   (0.35 -> 0.55 m at 0.20 m), which has no reciprocity justification.
   Inflating *pedestrian* radii as RVO2 sees them (`ped_margin`) leaves
   obstacle standoff **bit-identical** while still correcting the agent-agent
   mismatch. `ped_margin` adopted.

Had defect 1 gone unexamined, the comparison would have shown ORCA colliding
more than it should and credited SFM with a safety advantage that was an
artefact of a broken baseline.

### Frozen before the sweep

| Item | Value |
|---|---|
| Buffer semantics | A robot-side **safety buffer**, NOT a responsibility allocation. Inflating radii changes constraint geometry; it does not convert reciprocal half-responsibility into full. |
| Fairness invariant | Effective radius is **control only**. All collision and clearance outcomes use true body radii. Asserted in `py/orca.py`. |
| Selection rule | Smallest buffer whose development safe-success is within **5 percentage points** (absolute, fixed in advance) of the best configuration, with zero or minimal contact severity. Ordering: safe success -> deadlock -> contact severity -> **buffer size** -> proximity exposure -> runtime. |
| Rejected tie-break | "Least mean separation" — perverse, since it favours a controller operating closer to contact. |
| Sweep structure | Stage 1 margin-only screen {0, 0.02, 0.05, 0.10, 0.20}; stage 2 crosses only plausible margins with timeHorizon, timeHorizonObst, neighborDist, maxNeighbors. |
| Scope | Development seeds only. The selected buffer is frozen before confirmation seeds are touched. |

**Caveat carried forward:** the 2-9 mm deficit was measured in PAIRWISE
canonical cases. Dense crowds add multi-agent constraint interaction,
acceleration mismatch, curved pedestrian paths and simultaneous encounters, so
a buffer sufficient pairwise may fail at 48 agents. Canonical cases set the
search range; development trials decide.

### Baseline description for the manuscript

> An RVO2-based robot controller operating against common externally simulated
> pedestrian trajectories, with a tuned robot-side safety margin to account for
> non-reciprocal pedestrian behaviour.

No claim to standard ORCA collision guarantees in this mode — its reciprocity
assumption is not satisfied. The no-buffer reciprocal arm documents the
consequence **in this externally controlled pedestrian setup**, not the cost of
violating reciprocity in general.

### Stage-1 margin screen — result and two screen defects

Guided ORCA safe success rises 31.2% -> 93.8% across buffers 0 -> 0.20 m on 16
development scenarios. **The pairwise canonical deficit (2-9 mm) badly
understated what dense crowds require**; a buffer sized from canonical cases
alone would have been far too small.

Two problems with the screen itself, both to fix before stage 2:

1. **Contact rate is NOT monotonic in buffer size** — 1.12, 0.06, 0.25, 0.56,
   0.00 contacts/trial (raw counts 18, 1, 4, 9, 0). Plausible mechanism is the
   same exposure effect seen with the TTC filter: a more conservative robot
   lingers among pedestrians who do not avoid it. At n=16 the counts are too
   small to lean on. Stage 2 must not assume monotonicity.
2. **The selection tolerance was finer than the data resolution.** 16
   scenarios move safe-success in 6.25 pp steps against a frozen 5 pp
   tolerance, so only the exact best can qualify — the rule selected the
   LARGEST buffer (0.20 m) despite being written to prefer the smallest.
   Fixed by raising resolution (>= 40 development scenarios, <= 2.5 pp
   granularity), NOT by loosening the frozen criterion, which would be exactly
   the post-hoc fitting the freeze exists to prevent.

Unguided ORCA: 12.5% safe, 69-75% deadlock, 6-17 contacts/trial. The buffer
does not rescue it, consistent with its inability to route around obstacles.

### Balanced design

guidance {none, A*} x controller {SFM, ORCA}, with A*-SFM-TTC as an additional
SFM candidate rather than a fifth cell. Primary comparisons:

- SFM vs A*-SFM — guidance effect under SFM
- ORCA vs A*-ORCA — guidance effect under ORCA
- A*-SFM vs A*-ORCA — local-controller effect under common guidance
- A*-SFM-TTC vs A*-ORCA — strongest SFM candidate vs collision-aware reference

**Note:** ORCA with direct goal preference and no global route does not solve
maps requiring route selection (halts one radius from the obstacle face, never
reaching a goal behind it). This is a property of that configuration, not
evidence that global guidance is necessary for ORCA in general.

---

## TRAP THREAD — ARCHIVED, EXPLORATORY ONLY

**No positive trap-prediction or trap-area result is valid for manuscript use.**

Retained for provenance, not reuse: the original low-force implementation,
the component-interpolation correction, the exact-force checks
(`Check_DeadlockForce`), the invalid grid stability implementation, the
corrected equilibrium gate, the convergence observations, and every retraction
note above. Any number from this thread that is not explicitly marked SOLID is
withdrawn.

The manuscript may retain at most this compact methodological finding, in
discussion or limitations, never as a contribution:

> Grid-thresholded force magnitude did not reliably distinguish deadlocks from
> recoverable near-stalls, and gridwise stability classification was ill-posed
> because stability was evaluated away from equilibria and isolated equilibria
> were not reliably resolved by uniform sampling.

**Do not claim:** trap-basin area, percentage of hazardous workspace,
prospective trap prediction, attracting-region discrimination, or general
force-field deadlock diagnosis.

---

## 2b. Follow-up results — trajectory analysis and dose–response (1 Aug 2026)

Raw outputs archived before analysis (`v2/runs/v2/followup_manifest.json`).
`dose_response.py` and `sfm_explore.py` digests match the hashes frozen in
Amendment D. **Replay drift against the archived confirmation: 0.00e+00 across
all twelve cells** — the sweep measures the same system the confirmation did.

### Trajectory: exposure, not diversion (n = 37 per group, conditional)

All six prespecified components hold together:

| Component | Adverse | Stable | Difference |
|---|---|---|---|
| activation | 0.103 | 0.033 | +0.069 [+0.048, +0.091] |
| stopping (s) | 3.256 | 0.668 | +2.588 [+1.782, +3.378] |
| duration (s) | 7.244 | 2.485 | +4.760 [+3.141, +6.344] |
| route offset (m) | median 0.032, p90 0.054 | | < 0.35 m body radius |
| heading, nearest-point (deg) | median 0.069 | | < 15 deg |
| heading, arc-length (deg) | median 0.279 | | < 15 deg |

Contact occurs **on the original route**: median 0.02 m from the nearest point
of the unfiltered path (max 0.41 m). No scenario shows sustained route change
(median offset ≥ 0.35 m) and none shows a local excursion ≥ 1.0 m.

**The time-aligned heading difference is 53.3 deg and the place-aligned
difference is 0.07 deg.** Reporting the former alone would have supported a
false conclusion of controller-induced steering divergence. Diagnostics confirm
the place-aligned metric is sound here: zero backward matches, route
self-proximity 3.46 m against a 0.03 m deviation scale, and the two independent
matchings agree to 0.025 deg median.

Hausdorff is 8.6x the median offset (max 113x), i.e. **excursion-driven** —
exactly the reason it is never reported as the sole diversion metric.

### Dose–response (9000 runs, 500 scenarios x 2 planners x 9 configs)

Cite as **post-run corrected analysis**, not untouched frozen output — see
Amendment D.3 for the three-version audit trail. The correction changed 0 of 32
point estimates; 21 bootstrap intervals moved in the third decimal through a
shared-RNG stream-position artefact, and 48 estimates became readable that the
aborted version could not reach.

**L1 controller.** Activation breadth and earliness move as predicted for every
factor. Brake factor and speed-while-active came out COUNTER for `ttcHorizon`
and mixed for `ttcMinGap` — **a selection artefact, not a falsification.**
Activation is defined by `urgency > 0`, i.e. `t* < ttcHorizon`, so raising the
horizon enlarges the conditioning set with milder threats and dilutes the
conditional mean. Measured directly:

| Factor | Change in activation set | Conditional brake metric |
|---|---|---|
| `ttcStop` | −3.0% to +0.9% | interpretable → **consistent** |
| `ttcHorizon` | −21.6% to +15.7% | confounded → COUNTER |
| `ttcMinGap` | −29.1% to +50.2% | confounded → mixed |

A.5 prediction 3 specified "comparable activation episodes"; the instrument
averages over an activation set that the dose itself defines. So prediction 3
is **supported where testable (`ttcStop`) and untested elsewhere — not
refuted.** Testing it properly needs per-step urgency logging. Corroboration
that the pathway holds: unconditional mean speed falls monotonically with
horizon (1.097 → 1.056) while stationary time rises (3.42 → 3.99 s).

**L3 system.** Required wording: *no interior optimum in safe-goal success was
identified over the prespecified parameter ranges.* All peaks sit at a boundary
and none is separated from the endpoints by more than its interval. This is a
true null. **Do not infer that no optimum exists outside the tested range** —
the sweep bounds the claim to the levels actually run.

**L4 architecture, prediction 7.** Directionally consistent for all three
factors, but **the unguided onset is left-censored**: it fires at the weakest
level swept for every factor, so it lies at or below the swept range and is not
located. The honest statement is that the unguided arm is past the
mobility-cost criterion at *every* setting tested including the weakest, while
the guided arm reaches it only at `ttcMinGap=0.6`. Consistent with prediction 7;
does not localise the unguided onset.

### Separation floor — a parameter hypothesis, strictly exploratory

Paired within the same 500 scenarios (hybrid, safe success):

| Contrast | Δ | 95% CI |
|---|---|---|
| `ttcMinGap`=0.2 vs filter disabled | **+0.146** | **[+0.102, +0.190]** |
| default (0.4) vs filter disabled | +0.078 | [+0.034, +0.122] |
| `ttcMinGap`=0.2 vs default | +0.068 | [+0.038, +0.098] |
| `ttcMinGap`=0.6 vs filter disabled | −0.040 | [−0.086, +0.004] |

At `ttcMinGap=0.2` the lower bound (+0.102) exceeds the frozen smallest
meaningful effect (+0.075), which the confirmatory default did not reach.
**This is not a revised confirmation result and cannot be reported as passing
the primary endpoint.**

**Required manuscript wording** (a causal claim is not supported by this
design, and must not be made):

> In the post-confirmation dose–response analysis, a smaller TTC separation
> floor of 0.2 m produced the largest observed safe-goal improvement. Because
> this setting was selected after examining multiple configurations on the
> confirmed scenario set, the result requires independent confirmation.

> The follow-up suggests that the default separation floor may have been overly
> conservative and that the original result may be parameter-sensitive.

**Do NOT write** that the original "insufficient evidence" verdict *was caused
by* a badly chosen parameter. The sweep cannot establish that: 0.2 was selected
by comparing eight settings on the same 500 scenarios, so its apparent
advantage is inseparable from selection. Hypothesis and conclusion stay
separate.

An independent test of 0.2 requires new disjoint seeds, a fixed scenario
population, only the prespecified comparison, an analysis frozen before
results, and **no further parameter selection**. Primary estimand, chosen
before running — Question A, because it tests the actual new design claim
(reducing the floor improves the already-confirmed controller):

    p_safe(0.2) - p_safe(0.4)

with `p_safe(0.2) - p_safe(disabled)` reported as secondary. That ordering is
fixed here and not revisited after seeing results.

**This confirmation must not run before the reciprocal-pedestrian study.** The
exposure mechanism depends on SFM pedestrians continuing toward a slowed robot.
If reciprocal pedestrians largely remove safe→unsafe degradation, then
`ttcMinGap`=0.2 may be compensating for a model-specific artefact rather than
revealing a broadly preferable controller setting.

## 2c. ORCA-responsive pedestrian study (1 Aug 2026) — boundary of validity

Post-confirmation **model-generalisation** experiment (Amendments E, E.1). Not
an independent confirmation. Integrity clean: 3 arms x 500 (confirm, both
responsibility modes) and 3 x 300 (dev), no missing cells.

### Outcome rates (%), confirm set

| Arm | ORCA-responsive | | | ORCA-resp. + inflated radius | | |
|---|---|---|---|---|---|---|
| | safe | unsafe | deadlock | safe | unsafe | deadlock |
| disabled | 89.0 | 9.0 | 2.0 | 96.0 | 1.8 | 2.2 |
| ttc0.40 | 35.4 | 43.0 | 21.6 | 73.4 | 1.6 | 25.0 |
| ttc0.20 | 39.2 | 44.0 | 16.8 | 96.0 | 1.8 | 2.2 |

Primary, `p_safe(ttc0.40) − p_safe(disabled)`: **−0.536 [−0.582, −0.490]** —
entirely below zero. The confirmed SFM benefit does **not** reproduce.

### The contact result is dominated by responsibility allocation — not a robust TTC contact estimate

The two responsibility modes differ **only** by inflating the robot's radius
0.20 m in the pedestrians' view. That single change takes safe→unsafe
transitions from **191 to 0**.

**These 191 runs are NOT invalid.** They are real outcomes of the exact
asymmetric ORCA-responsive model implemented. Calling them an "artefact"
without qualification would wrongly imply the runs are broken. What is true is
that the model they belong to is not a faithful full-reciprocity experiment.

**Required wording:**

> The standard-radius ORCA-responsive configuration produced numerous
> marginal-clearance failures that disappeared when the robot's perceived
> radius was inflated. This sensitivity indicates that the result is dominated
> by asymmetric responsibility allocation rather than providing a robust
> estimate of TTC contact risk under fully reciprocal navigation.

**Forbidden:** "TTC causes contacts under reciprocal pedestrians", and equally
"the 191 contacts are an artefact" unqualified.

Mechanism: standard ORCA splits avoidance 50/50; the robot does not run ORCA, so
when TTC stops it, it contributes none of its half and pedestrians clip it —
the limitation stated in Amendment E before the run, now measured.

### What IS robust across both responsibility modes

1. **The deadlock cost persists.** 21.6% and 25.0% versus 2.0% and 2.2%
   disabled. Responsibility allocation does not rescue it.
2. **No safe-success benefit appears under either mode.** Safe success falls in
   both.
3. **Why, and this is the substantive finding:** under SFM pedestrians the
   unfiltered hybrid arrived unsafely in **62.8%** of runs — a large pool for
   TTC to rescue. Under ORCA-responsive pedestrians the unfiltered arm is
   already 89.0% / 96.0% safe with only 9.0% / 1.8% unsafe arrivals. **There is
   almost nothing left to rescue, so only the mobility cost remains.**

> The confirmed TTC benefit is contingent on a high baseline unsafe-arrival
> rate, which is itself a property of the pedestrian model. Where the crowd
> model already avoids the robot, predictive braking has little to recover and
> its deadlock cost dominates.

This **bounds the original claim** rather than overturning it, and it is the
boundary-of-validity result the study was run to obtain.

### The 191 adverse transitions are millimetre-scale boundary crossings

Replayed with closest-approach captured (`min_ped_gap`, surface-to-surface,
negative = penetration):

**The first pass reported these as a uniform "−0.001 m" with IQR
[−0.001, −0.000]. That was a `%.3f` print artefact, not a measurement** — any
value between 0.5 and 1.5 mm renders identically at three decimals. Recomputed
at full precision, the depths vary genuinely:

| Penetration depth | value |
|---|---|
| minimum | 0.0000045 m (4.5 µm) |
| p25 | 0.000281 m |
| **median** | **0.000502 m (0.50 mm)** |
| p75 | 0.000641 m |
| **maximum** | **0.001363 m (1.36 mm)** |
| distinct values | **191 of 191** |
| max/min ratio | **306×** |

No simulator ceiling exists — the engine applies no clipping and no position
correction, and 191 distinct depths spanning 306× confirm it. The correct
statement is therefore:

> All 191 safe→unsafe transitions involved maximum penetrations below 1.4 mm,
> with a median of 0.50 mm.

**Not** "all are 1 mm", which was an artefact of reporting precision.

Corroboration from an independent direction: for undamped unit-mass contact
overlap ≈ v/√kBody, so these depths imply contact speeds of **1.4 mm/s (p10) to
10.5 mm/s (p90)** against a nominal 1300 mm/s — the robot is effectively
stationary at contact, exactly as the braking-exposure account requires.

For comparison, the same runs without the filter clear by about 2 mm, and
safe→safe controls clear by 280 mm (median).

**Do not write "191 failures"** — under the frozen binary definition these are
unsafe, but they are shallow contacts, and "failure" implies the trial ended
catastrophically. It did not; these runs reached the goal.

**Do not claim ORCA generally permits zero clearance** as a universal property
of the algorithm. The supportable statement:

> In the tested ORCA-responsive implementation, trajectories frequently
> approached the collision-constraint boundary, making the zero-gap binary
> endpoint highly sensitive to millimetre-scale differences.

(Controls reach p10 = 0.000 m, so boundary-adjacent travel is common here.)

**This is exactly what the frozen method decisions anticipated** (§5: *contact
duration and maximum penetration are co-primary; episode counts are secondary
and ambiguous*). By the co-primary penetration endpoint these contacts are
negligible; by the binary count endpoint they are 191 failures. **Report both,
lead with penetration.**

Timing: 189/191 contacts occur after sustained activation, 2 before, 0 without
activation — consistent with the exposure account within this model.

### Radius inflation is NOT a clean control

It does not merely remove grazes. On the same 191 scenarios it also changes:

| Quantity | standard | inflated | change |
|---|---|---|---|
| min ped gap (m) | −0.000 | +0.201 | +0.202 |
| stop time (s) | 5.098 | 2.689 | **−2.408** |
| activation | 0.114 | 0.065 | **−43%** |
| duration (steps) | 7078 | 6143 | −935 |
| deadlock rate | 0.216 | 0.250 | **+0.034** |

Pedestrians giving more room means the robot brakes less and finishes sooner —
yet deadlock **rises**. The two responsibility modes are materially different
dynamics, not one endpoint threshold apart. Neither can be treated as a
correction of the other, and conclusions are bounded by responsibility
allocation exactly as E.1 required.

### The ceiling, measured rather than inferred

TTC can raise safe completion only by converting unsafe arrivals. The size of
that pool is a property of the **pedestrian model**, not of the filter:

| Pedestrian model | P(unsafe \| TTC off) | pool | →safe | →deadlock | stayed | **max achievable gain** |
|---|---|---|---|---|---|---|
| SFM | 0.628 | 314 | 83 | 55 | 176 | **+0.628** |
| ORCA-responsive | 0.090 | 45 | 7 | 14 | 24 | **+0.090** |
| ORCA-resp + inflated radius | 0.018 | 9 | 0 | 1 | 8 | **+0.018** |

"Max achievable gain" is the improvement if the filter converted *every* unsafe
arrival and broke nothing. Under the inflated-radius model TTC could gain at
most **1.8 points** while costing **22.8 points** of deadlock. **The comparison
was unwinnable before the filter did anything** — this is a ceiling effect, not
a controller failure, and it must be presented that way.

### ORCA neighbour cap — bounds the dense condition

`max_neighbors = 10`; 60 scenarios replayed, 15 per condition:

| Condition | pedestrians | cap-bound % of (ped, step) | max neighbours seen |
|---|---|---|---|
| baseline | 48 | 1.15 | 14 |
| bigObs | 48 | 1.25 | 14 |
| cluttered | 48 | 0.27 | 13 |
| **dense** | **80** | **6.21** | **27** |

Overall 2.70%; spread across conditions 5.93 pp. The cap binds **~5x more often
in dense**, where a pedestrian can have up to 27 agents inside the 5 m
neighbour radius but ORCA constrains against at most 10. **Dense-condition
conclusions are bounded by this.** It does not explain the inversion — the
ceiling does — but a larger-value check would be informative, and would be
reported as MODEL SENSITIVITY, never substituted into the frozen study.

### Attribution and remaining caveats

Per E.1, differences are attributable to replacing the SFM pedestrian dynamics
with the specified ORCA-based model — **not** specifically to yielding.
Replacing the model also changes anticipation, speed selection, collision
geometry and congestion propagation.

Development set (n=300) reproduces the pattern: disabled 89.3 safe, ttc0.40
37.3 safe, deadlock 2.0 → 22.0. Robustness set only; it informed earlier design
work and is not a second confirmatory sample.

`ttcMinGap`=0.20 remains a frozen secondary. Under ORCA-responsive pedestrians
it is +0.038 [+0.010, +0.066] over 0.40 — same direction as the SFM sweep, but
external-model evidence for a previously selected candidate. It does not
replace an independent seed-disjoint SFM parameter confirmation.

## 2d. AUDIT FINDING — PROTOCOL §5a WAS NEVER IMPLEMENTED (3 Aug 2026)

Raised while checking whether Amendment F's recovery layer would collide with
the existing blockage handler. **There is no existing blockage handler.**

`PROTOCOL.md` §5a "Blockage handling — FROZEN" specifies: declare lack of
progress when distance to the active waypoint improves by < 0.05 m over a
continuous 3 s window; advance to the furthest later waypoint with verified
collision-free line of sight; else run fresh A\* from the current position;
after 2 recovery attempts classify the trial `blocked`.

**None of it exists in either engine.** No match for blockage / replan /
recovery / attempt in `py/sfm_explore.py` or `v2/RunTrial.m`. There is no
`blocked` outcome category anywhere in the results.

### What the engines actually do

- **Waypoint progression only**: advance to the next waypoint when within
  `rWaypoint` = 2.0 m. This is normal progression, not blockage recovery — it
  is not triggered by lack of progress and cannot skip to a *later* visible
  waypoint.
- **Deadlock**: speed < `stallSpeed` 0.05 m/s continuously for `stallSteps`
  500 = **5.0 s** while outside the goal radius. Not the §5a 3 s / 0.05 m
  progress rule, and no recovery attempt precedes it.

### Consequences — three, and the first is a manuscript correctness issue

1. **Every result to date was produced by the strict one-shot, no-recovery
   variant.** Confirmatory, dose–response, trajectory and ORCA-responsive
   studies alike. The manuscript must describe the planner as **one-shot A\*
   with no event-triggered recovery**, and must NOT claim the §5a behaviour.

   Stated precisely, because the framing matters: the experiments did **not**
   violate the code they ran. The written protocol contained an intended
   feature that was never built; the experimental system was consistent across
   every arm; and all results remain valid for the strict one-shot
   implementation they actually describe. The manuscript follows the code that
   ran, not the unexecuted specification.
2. **The three-way ablation §5a demands has never been run** (strict one-shot /
   safe waypoint advancement / event-triggered A\* recovery). It is an
   outstanding protocol commitment, now explicit rather than assumed done.
3. **Amendment F has no interaction to control for.** Nothing needs enabling or
   disabling across arms, because no blockage handler exists in any arm. This
   must be *stated* in F rather than left implicit — the reason there is no
   confound is that the mechanism is absent, not that it was balanced.

Recorded as a spec/implementation divergence, not a defect in any result: the
deadlock numbers mean exactly what the 5 s stall rule says, and all arms are
affected identically.

## 2e. External transfer study — CrowdNav (Amendment H, 4 Aug 2026)

960 runs, integrity clean: 480 complete pairs, no missing cells. **Transfer
check, not independent confirmation.** Arms are *direct-goal* SFM ± TTC —
CrowdNav has no static obstacles, so A\* degenerates to a straight reference and
**the guidance x TTC interaction is untestable here**.

### PRIMARY — robot invisible, dt = 0.25 s (n = 200 pairs)

| Outcome | Δ (TTC − reference) | 95% CI |
|---|---|---|
| collision (unsafe) | **−0.110** | [−0.180, −0.040] |
| goal completion | **+0.065** | [+0.005, +0.130] |
| CrowdNav timeout | +0.045 | [+0.020, +0.075] |
| **our 5 s deadlock** | +0.045 | [+0.020, +0.075] |
| min clearance | +0.052 m | [+0.033, +0.071] |
| traversal time | +2.08 s | [+1.31, +2.89] |

**The trade-off transfers**: collisions fall, immobility rises. Per the frozen
ordered interpretation this is the "trade-off transfers" branch.

**One difference from the original that must not be smoothed over.** In the
confirmatory study TTC *reduced* goal arrival (−0.120). Here it *increased* it
(+0.065). The safety benefit transfers; the mobility cost appears as deadlock
and slower traversal rather than as lost arrivals. The pattern is similar, not
identical, and should be reported as such.

### ABSOLUTE PERFORMANCE IS POOR — state this plainly

The reference arm collides in **83%** of episodes (TTC arm 72%). CrowdNav's own
ORCA baselines achieve near-zero collisions. This is **not** a competitive
navigation result, and the study must not be presented as one. It is consistent
with the known finding that SFM repulsion is too weak to prevent contact (goal
drive 2.60 vs repulsion 2.10) against pedestrians that do not yield. Only the
paired CONTRAST is interpretable.

### SENSITIVITY — robot visible: the effect REVERSES

| Outcome | Δ | 95% CI |
|---|---|---|
| collision | **+0.335** | [+0.250, +0.420] |
| goal completion | **−0.375** | [−0.460, −0.285] |
| deadlock | +0.045 | [+0.015, +0.080] |

When ORCA humans include the robot in their avoidance, **TTC becomes harmful**.
This converges with Amendment E, where the benefit vanished under
ORCA-responsive pedestrians. Two independent routes now indicate the confirmed
benefit depends on pedestrians that do not yield.

### SENSITIVITY — dt = 0.01 s (seeds 0–9, prespecified)

Invisible: collision −0.275 [−0.425, −0.125] — **larger** than the primary
−0.110. Visible: collision 0.000, deadlock +0.350 [+0.175, +0.525].

**Measured, not assumed:** the slew limiter clipped on **0** steps across 200
primary runs at dt = 0.25, and on **681** steps across 40 runs at dt = 0.01. The
within-step brake ramp is genuinely absent at the native interval, exactly as
D1 anticipated — and the TTC benefit is larger when it is present. CrowdNav's
coarse control interval **attenuates** the effect.

### Follow-up diagnostics on the reversal

**Timeout and our deadlock are near-identical sets here.** Invisible: 9 runs,
both labels, no singletons. Visible: 11 both, 1 timeout-only, 2 deadlock-only.
In this benchmark the two are largely redundant — but not exactly, so both are
still reported.

**The reversal is uniform across strata**, so no single condition drives it:

| Stratum | Δ collision | Δ goal |
|---|---|---|
| circle 5p | +0.280 | −0.300 |
| circle 10p | +0.360 | −0.360 |
| square 5p | +0.340 | −0.400 |
| square 10p | +0.360 | −0.440 |

**Transition decomposition, visible mode:** 84 goal→collision against only 16
collision→goal. The reversal is TTC converting successful runs into contacts,
not failing to rescue.

**Mechanism — POST-RUN EXPLORATORY, not a frozen test.** This diagnostic was
written after the transfer outcomes were seen. It is convergent evidence from an
independent simulator, not another confirmatory result, and must be labelled
that way.
Of 40 sampled goal→collision cases: **39/40 contacts occur after sustained TTC
activation**, 1 before, 0 without activation. Mean speed in the final second is
**0.197 m/s** against a 1.0 m/s preferred speed; mean activation fraction 0.454.

The robot is braked to near-standstill and contacted — the pattern seen in the
original safe→unsafe transitions and again in Amendment E, now appearing in an
independently implemented simulator with a different pedestrian model.

**Required wording:** *three complementary analyses produced convergent evidence
consistent with braking-induced exposure.* **Do NOT write** that three settings
"prove" the mechanism, or that the convergence is confirmatory. Only the first
analysis was frozen in advance.

### D3 speed cap barely binds

2 clipped steps across 200 primary runs; max requested 2.37 m/s against the
1.54 m/s cap. The mapping is not quietly doing work.

## 2f. Execution environment and compute provenance (for Methods)

Captured by `py/capture_provenance.py` -> `v2/runs/v2/provenance.json`,
regenerated from the machine rather than transcribed.

### Hardware and toolchain

    Intel Core i9-9880H @ 2.30 GHz, 8 physical / 16 logical cores, 16 GB RAM
    macOS 26.5.1 (x86_64)
    Python 3.13.2 (CPython); numpy 2.3.1, scipy 1.16.3, matplotlib 3.10.8,
      pytest 9.0.2, gym 0.26.2, pyrvo2 (unversioned)
    MATLAB R2025b Update 5 (no toolboxes required)
    CrowdNav commit 20d678085c06831e658a65b9e20c8bb6f6ecdc10 (vita-epfl)

### Archived simulation runs

| Study | Runs | Stack | Workers |
|---|---|---|---|
| confirmatory factorial | 2000 | MATLAB + Python | — |
| dose-response sweep | 9000 | Python | 12 |
| ORCA-responsive pedestrians | 3900 | Python + RVO2 | 12 |
| CrowdNav external transfer | 960 | Python + CrowdNav | 10 |
| recovery Stage A calibration | 3200 | Python | 10 |
| **total** | **~19 060** | | |

Plus 160 calibration scenarios generated in MATLAB, and roughly 1 500 replay
runs across the trajectory, penetration-precision and integration-diagnostic
analyses.

### Wall-clock timing — an honest gap

**Wall clock was not instrumented from the start.** Only the ORCA-responsive
study has a measured figure (approximately 6 h at 12 workers; the RVO2 solve
dominates). The others completed before timing was recorded, and
`provenance.json` stores `null` for them rather than an estimate.

A plausible-looking number in a methods section is worse than a stated gap,
because a reader cannot distinguish the two. If the manuscript needs per-study
timings, the runs must be re-executed under timing, not back-filled.

Useful scale figures that ARE reliable: a single guided trial is ~15-20 s at
dt = 0.01 s with 48 pedestrians, and a deadlocking trial runs the full 15 000
steps at ~35-40 s. ORCA-responsive trials are slower again because RVO2 solves
every step for every pedestrian.

## 2g. Recovery Stage A calibration — result and two warnings (5 Aug 2026)

3200 runs, 160 calibration scenarios x 20 configurations. **Calibration only.
Nothing here may be presented as the formal recovery test.**

Selected by the frozen ordered rule: **`w4.0_p0.3_a0.8`** (window 4.0 s,
required progress 0.30 m, activation fraction 0.80). All 18 configurations
passed both calibration screens, so selection fell to the ranking: lowest
deadlock, then fewer episodes, then shorter release, then weaker intervention.

| Arm | safe | unsafe | deadlock |
|---|---|---|---|
| reference (no TTC) | 0.388 | 0.594 | 0.019 |
| TTC only | 0.431 | 0.425 | 0.144 |
| **TTC + recovery (selected)** | **0.444** | **0.438** | **0.119** |

Recovery fires in 86/160 scenarios; it is not inert.

### WARNING 1 — the deadlock reduction is below the frozen Gate 3 threshold

Recovery cuts deadlock by **2.5 points** (0.144 -> 0.119) on the calibration
set. **Gate 3 requires at least 5 points.** On this evidence the controller as
designed may not clear its own efficacy gate.

This is calibration, not evidence, and the calibration set is small — but it is
an early signal that should be stated now rather than discovered after the
evaluation. **The margin must NOT be relaxed to accommodate it.** If the
evaluation fails Gate 3, the correct report is that a progress-aware release of
the discretionary brake did not deliver a worthwhile deadlock reduction.

Note also that unsafe arrival rose 1.3 points (0.425 -> 0.438), well within the
5-point calibration screen but consuming half of Gate 1's 2.5-point evaluation
margin.

### WARNING 2 — `theta_prog` is nearly non-discriminating

Configurations differing ONLY in the required-progress threshold produced
identical results in 160/160 scenarios at `w4.0_a0.8`, and differed in just
4/160 at `w2.0_a0.6`.

The parameter is not inert in code — it does change behaviour occasionally —
but it fails to discriminate in this regime, because progress is effectively
bimodal: a robot under sustained braking makes essentially zero progress over
the window, far below even the 0.10 m threshold, while a moving robot far
exceeds 0.30 m. Values between those extremes are rarely realised.

Consequences:

1. The selected `p0.3` is one of a tied set. The frozen rule broke the tie
   deterministically toward the weaker trigger, which is what it was designed
   to do, but the choice carries little information.
2. **Stage B should not spend its budget on the trigger.** The action
   parameters — release magnitude, maximum release duration, episode cap — are
   where the remaining leverage is.
3. The manuscript must report that the progress threshold had little effect
   over the swept range, rather than presenting it as a tuned quantity.

## 2h. Computational cost — captured for the discussion, not yet committed

Profiled 6 Aug 2026; raw data in `v2/runs/v2/compute_cost.json`. **Undecided
whether this enters the paper.** Recorded now because the measurement is cheap
and reconstructing it later would be guesswork.

Reviewer 1.29 asked for graph construction and A\* query times to be reported
separately, so some computational content is required regardless; this is the
simulation-side counterpart.

### Where the time goes (one guided trial, 6216 steps, 62.2 s)

| Function | % of total | Calls |
|---|---|---|
| `compute_forces` | 88.6 | 6 216 |
| `poly_signed_distance_many` | **68.6** | **62 160** |
| `np.roll` (inside the above) | **27.1** | **497 280** |

The bottleneck is **obstacle** distance, not pedestrian interaction: 10
obstacles x 6 216 steps, each call doing small operations on 49 agents against
a ~6-vertex polygon.

### GPU acceleration would not help, and would likely be slower

Worth stating in the discussion because it is a natural reader question:

1. **Arrays are tiny.** Kernel launch overhead (5-20 us, several kernels per
   call) exceeds the arithmetic it would replace.
2. **The integrator is sequential.** Semi-implicit Euler makes step $n{+}1$
   depend on step $n$; there is no timestep parallelism.
3. **Parallelism is already on the right axis** — independent trials run
   concurrently across CPU workers.
4. No CUDA device is present on the machine used (AMD Radeon Pro 5500M).

Batching many trials as tensors would amortise launch overhead, but that means
re-implementing the engine with batched semantics in torch or JAX — a separate
validated-infrastructure project that would break bit-exact agreement with the
MATLAB reference.

### A real optimisation exists but was deliberately NOT applied

`np.roll` is called 497 280 times per trial, recomputing an unchanging polygon
edge rotation on every call. Precomputing edges once per scenario is a
plausible 1.5-2x CPU speedup.

**Not applied**: any change to `compute_forces` invalidates the cross-language
validation chain — bit-exact MATLAB agreement, archived hashes, the
`verify_confirm` gate — and completed studies would need re-running for
consistency. A 2x speedup does not justify re-validating the foundation of the
paper mid-write. Recorded as future work.

**Recommended scaling if throughput ever matters:** more CPU cores. 32-64 cores
gives 3-6x with no code change and preserves determinism exactly.

## 2i. Recovery Stage B — futility passed, but no candidate can clear both gates

8960 runs, 12.55 h wall clock (timed; the earlier studies were not).
**Calibration only.**

### The result

The futility rule **passed**: 18 candidates reduce calibration deadlock by at
least 3 points. The frozen ranking selects `T1_r0.8_d5.0_c3`
(release 0.8, max duration 5 s), at $-5.6$ points deadlock.

**But that candidate raises unsafe arrival by $+4.4$ points, and Gate 1 permits
$+2.5$.** The full frontier, with raw scenario counts at $n=160$ (one scenario
= 0.625 pp):

| release / duration | Δ deadlock | scen | Δ unsafe | scen | Gate 3 | Gate 1 |
|---|---|---|---|---|---|---|
| 0.8 / 5.0 s | −0.056 | −9 | +0.044 | +7 | **yes** | **NO** |
| 0.8 / 3.0 s | −0.044 | −7 | +0.031 | +5 | no | NO |
| 0.8 / 2.0 s | −0.037 | −6 | +0.025 | +4 | no | NO |
| 0.6 / 5.0 s | −0.025 | −4 | +0.006 | +1 | no | **yes** |
| 0.6 / 3.0 s | −0.025 | −4 | +0.013 | +2 | no | yes |
| 0.4 / 2.0 s | −0.006 | −1 | +0.006 | +1 | no | yes |

**Candidates meeting both Gate 3 and Gate 1: zero.**

### What this means

The recovery layer has its own internal safety--mobility trade-off, along the
release axis. Releasing the brake enough to cut deadlock meaningfully costs
more unsafe arrivals than Gate 1 permits; staying within Gate 1 leaves the
deadlock reduction at roughly half the target. This is the same trade-off the
paper documents in the TTC filter itself, reappearing one layer up.

**A gap in the futility rule, stated plainly.** The rule required $\geq 3$
points of deadlock reduction *within the calibration screens*, and the
calibration safety screen is the loose $+5.0$ points, not Gate 1's $+2.5$. So
the rule passes a candidate that is already predictable to fail Gate 1. The
rule is not revised --- it was frozen --- but its verdict must not be read as
"proceed with confidence".

### Second inert parameter

`cap` produced **1 of 3 distinct behaviours**: episode counts are 0 in 74
scenarios and 1 in 86, never 2 or more, so caps of 1, 2 and 3 are equivalent.
Recovery never gets a second episode because re-arming requires 2 s of
demonstrated restored progress, which a stalled robot rarely achieves.

Two of the five calibrated parameters --- `theta_prog` and `max_episodes` ---
turned out non-discriminating in this regime. Both must be reported as such
rather than presented as tuned quantities.

### Trigger robustness could not be exercised

Mean $|\Delta$ deadlock$(T_1) - \Delta$ deadlock$(T_2)|$ over 27 actions:
**0.0000**. As stated before the run, $T_1$ and $T_2$ differ in 1 of 160
scenarios by construction, so this is not evidence of robustness. The frozen
ranking failed to identify two meaningfully distinct top triggers.

## 3. Defects found in the submitted implementation

No reviewer caught these. Reporting them unprompted is strong response-letter material.

1. **Repulsion vanished at contact.** Magnitude ∝ d·exp(−d), and with r_inf = 1 the active range was exactly where that function *increases* — weakest at contact, strongest at the cutoff.
2. **Obstacle penetration was self-reinforcing.** `p_poly_dist` returns negative distance inside a polygon; the legacy code fed that into both the cutoff test and the direction vector, pushing a penetrating agent deeper.
3. **Goal attraction scaled as distance².** Raw displacement instead of a unit vector.
4. **The integrator was mislabelled and unstable.** Advertised semi-implicit Euler; actually advanced position with the previous velocity plus ½Fdt². On a harmonic oscillator over 637 periods: relative energy spread 2.0e−2 (correct scheme) vs 2.3e+17 (legacy). **The velocity clamp to [1,4] masked this** — unstable dynamics rendered as motion pinned at v_max.
5. **v_min = 1 m/s meant nothing could ever stop**, including all 48 pedestrians.
6. **Pedestrian goals were never updated** — the crowd degenerated into agents jittering around fixed points.
7. **Both robots shared one simulation** and exerted forces on each other and the crowd; "identical conditions" did not hold.
8. **The 6000-step horizon was ~1.1× the free-space traverse time**, so timeouts partly measured the clock.
9. **`StatisticalTests.m` depended on the paid Statistics Toolbox** (`ranksum`), so published results were not reproducible on base MATLAB.

## 4. Errors made and caught during THIS revision

Belongs in the methods/limitations discussion — it documents why the corrected analysis is trustworthy.

| Error | How it was caught |
|---|---|
| Robot repulsion gain applied to every agent pair, so raised-gain arms silently changed the whole crowd and were not matched-seed comparisons | Audit prompted by a reviewer-style question about pedestrian dynamics. Invalidated a 90% safe-success figure, corrected to 55%. Now guarded by `Test_GainAsymmetry`. |
| 2×2 position Jacobian used to classify attracting equilibria | The system is second order; correct criterion is the 4×4 [0 I; A −I/τ]. Verified against direct simulation: the 2×2 test gave false positives in 2 of 5 rotational cases. |
| TTC smoothing reintroduced the low-speed blind spot by ramping the approach gate over closing *speed* | Mandatory head-on regression test failed (−3.5 mm penetration). |
| Interpolating the magnitude field \|F\| instead of components | Physically impossible reading (3–6 m/s² at a stalled robot). Exact evaluation gave ~0.001 m/s². Error of ~3 orders of magnitude, and it produced a confident but wrong FAIL verdict. |
| Hard negatives were all the START position: the robot begins at rest, so the minimum-speed point is step 1 (v0/tau*dt = 0.026 m/s) unless tracking is gated past the acceleration phase. Produced an apparently strong AUC 0.978 that actually measured "is the deadlock somewhere other than the start corner" | Identical 0.026 m/s minima across seven of nine negatives |
| First trap test was circular — field evaluated at the stall moment | A stalled robot must sit at a force equilibrium by Newton's second law. Replaced with the pre-run static field. |
| Emergency stop threshold (0.05 m) sat *below* the near-collision boundary (0.10 m) | Controller was designed to halt only after entering the band scored as unsafe. Now asserted in `SfmParams`. |

| Analysis looked up `ttcStop=0.8` and `ttcMinGap=0.4`, labels that do not exist: the sweep runs the three factors' shared centre **once** under `ttcHorizon=2.0` rather than simulating the same 500 scenarios three times. The fallback branch would have printed "configurations missing — skipped" and **silently reported only one of the three factors** | The one-factor-at-a-time assertion added for a different reason (checking the levels were not bundles) fired first and named the two missing labels. Recorded as a manifest amendment, not a silent overwrite |
| A regression test that did not catch the bug it was written for. `Test_AnalyzeDose` asserted the goal-arrival row was "not identically zero", but with the bug live the row reads `0.000 0.000 0.000 0.000 nan`, and `nan != 0.0` is True — the NaN rank correlation was counted as a non-zero rate, so the test passed while the defect was present | Reintroducing the bug deliberately and confirming the test still passed. Fixed to ignore non-finite values; now verified to fail with the bug and pass without it. **A test that has never been observed to fail is not evidence of anything** |

### Computational-method changes (NOT results)

Distinct from the table above. Nothing here reflects on the behaviour of the
system under study; each is a change of tool made because an implementation was
impractical at the data size involved.

- **Discrete Fréchet → Hausdorff + time-aligned separation, for trajectory
  comparison.** The Fréchet implementation is a Python double loop over a
  ~1500 × 1500 cost matrix per scenario pair and did not complete a single
  scenario in 18 minutes of CPU; it was killed (exit 144). **The exit status
  carries no information about trajectory behaviour** and must never be
  reported as though it did.

  The substitution is **not** an equivalence. Hausdorff distance measures
  whether two trajectories occupy similar spatial *regions*; Fréchet
  additionally respects traversal *order*. In the replacement, Hausdorff
  carries the spatial-deviation question alone, while time-aligned separation
  and heading differences carry the temporal and directional questions Fréchet
  would have partly covered. Manuscript wording must be:

  > Hausdorff distance assesses spatial route deviation, while time-aligned
  > separation and heading differences assess temporal and directional
  > divergence.

  Recurring pattern worth stating in the limitations: this is the fourth time a
  textbook-correct method was adopted without costing it against the actual
  data size (`Graph.m` at 225k point-to-polygon calls; `TrapField` at 38k
  `ComputeForces` calls; the pure-Python obstacle loop). Each was caught only
  by something failing to finish, never by inspection.

### Scope limit on the trajectory analysis

The 111 scenarios are **selected by outcome transition**. That is the correct
sampling for *explaining* the confirmed transitions, and it is the wrong
sampling for estimating *how often* diversion occurs. The analysis is
conditional and exploratory; a population-level diversion rate would require
all 500 pairs. The comparison groups are matched on size only — not on
condition or crowd density — so between-group contrasts are descriptive and
use an unpaired bootstrap, unlike the paired confirmatory contrasts.

**Metric cautions, each with a check in `py/diversion_diagnostics.py`:**

- **Hausdorff is a maximum**, so one brief local excursion can set it while the
  routes are otherwise identical. It is never reported as the sole diversion
  metric — always beside the **median** nearest-route deviation (which a single
  excursion cannot move) and the place-aligned heading. The ratio of the two is
  reported so a reader can see when Hausdorff is excursion-driven.
- **Unequal duration.** The braked run lasts longer and the time-aligned
  metrics truncate to the shorter run, discarding the filtered run's tail —
  where a late contact would occur. The fraction truncated is reported, as is
  the count of contacts occurring after the unfiltered run ended (for which
  time-aligned separation is undefined but the offset-to-route metric still
  applies). This is why both are reported.
- **Self-intersecting routes.** Nearest-point matching can pair two points
  adjacent in space but far apart in traversal. Diagnosed directly (does the
  matched index run backwards?) and cross-checked against **arc-length
  matching**, which is monotone by construction and cannot make that error,
  though it instead assumes comparable route lengths. The two matchings make
  opposite trade-offs, so agreement between them is evidence neither artefact
  drives the result.
- **Time-aligned heading is confounded by lag** and cannot indicate diversion
  on its own — see the place-aligned metric below.

Prespecified interpretation, recorded before the group results were read:

| Pattern | Reading |
|---|---|
| small Hausdorff + small aligned separation + small heading change | lingering on essentially the same route (exposure) |
| small Hausdorff + large aligned separation | similar geometry, different timing |
| large Hausdorff | genuine local diversion |
| large heading deviation near activation | controller coupling, even with path length unchanged |

## 5. Frozen method decisions

Full detail in `PROTOCOL.md`. Highlights that must appear in the manuscript:

- 1 grid cell ≡ 1 m; 50 × 50 m workspace; robot radius 0.35 m; pedestrian 0.30 m.
- Unit mass throughout, so all force terms are accelerations in m/s².
- Success = within 0.5 m **and** speed ≤ 0.1 m/s **and** held 1.0 s **and** no contact during dwell. Sensitivity reported at 0.25 / 0.5 / 1.0 m.
- Primary endpoint is **safe success**, not goal success.
- Contact duration and maximum penetration are co-primary; episode counts are secondary and ambiguous.
- Smoothness reported as **RMS** jerk; proximity as a **fraction** of trial duration.
- Non-overlapping obstacles; all rejected maps counted and reported.
- Pedestrian model, initial states, goals and random draws matched across arms; **realised pedestrian trajectories may still differ** through reciprocal interaction with the method-dependent robot trajectory.
- `aPed` governs pedestrian–pedestrian repulsion; `aPedRobot` is the robot's controller gain. Deliberately asymmetric — a behavioural asymmetry, not a physical one. Contact term stays symmetric.
- Wilson intervals for proportions, percentile bootstrap for means, Holm–Bonferroni across swept conditions, every effect size named.

## 6. Positioning

**Paper identity:** a mechanistic and comparative analysis of progress and safety
failures in globally guided social-force navigation. **Not** a novel algorithm.

### The story, as it now stands

A\* + SFM is still the system — but it is the **platform**, not the claim. The
paper is no longer "we combined A\* and SFM". The object of study is what
happens when a predictive safety filter is layered on top.

    A* global route -> SFM local controller -> TTC safety filter -> robot command

Each layer answers a different question: A\* *which way around the obstacles*,
SFM *how to move through nearby people*, TTC *whether to brake for an
approaching collision*. **The layers do not add independently**, and that is the
result:

- Without guidance, TTC mostly makes the robot more conservative and raises
  deadlock, with **no detectable improvement in safe completion**
  (−0.004 [−0.024, +0.016]).
- With guidance, TTC cuts unsafe arrivals enough for a confirmed net gain
  (+0.078 [+0.034, +0.122]) — while still causing substantial deadlock and
  degrading some already-safe runs.

**Headline claim, scoped to what was tested:** *under the tested SFM crowd
dynamics and benchmark conditions, global guidance determines whether TTC
filtering yields a net safe-completion benefit or primarily increases
immobility.*

The unscoped version — "global guidance determines whether predictive collision
filtering produces useful safety or merely immobilises the robot" — is **too
universal** and must not be used. The factorial established this under one
pedestrian model. Only if the ORCA-responsive study reproduces the pattern may
the claim be broadened, and then only to "persists across both tested
pedestrian models".

The four-arm factorial is what licenses this, and is why the paper is stronger
than a "our hybrid beats the baseline" comparison:

| System | A\* | SFM | TTC |
|---|---|---|---|
| SFM | no | yes | no |
| SFM–TTC | no | yes | yes |
| Hybrid | yes | yes | no |
| Hybrid–TTC | yes | yes | yes |

**Mechanism, stated conditionally.** The trajectory analysis sampled selected
transition classes — chiefly the 37 safe->unsafe cases — so the claim is bounded
to them:

> In the examined safe->unsafe transitions, TTC did not produce sustained route
> diversion; the robot remained close to the unfiltered route while showing
> greater activation, stopping and duration, consistent with braking-induced
> exposure.

**Do not write** that TTC "holds the robot on nearly the same route" as a
general property. The analysis is conditional and cannot speak to prevalence
across all 500 scenarios.

That a *single* mechanism explains both rescues and degradations remains a
**hypothesis**: the rescue trajectories were not examined the same way. The
defensible form:

> The same braking intervention plausibly underlies both outcomes: it can
> interrupt imminent unsafe motion, but prolonged intervention can increase
> exposure and deadlock.

**Working title:** *When Does Predictive Safety Filtering Help? A Factorial
Study of Global Guidance, Social-Force Navigation, and TTC-Based Collision
Avoidance.*

**Central claim, for the abstract:** A\*–SFM provides the global–local
navigation platform. The study isolates the effect of adding TTC filtering and
shows that predictive safety intervention is **architecture-dependent**: under
SFM crowd dynamics it improves net safe completion when global guidance is
available, but offers no detectable benefit without guidance. The benefit is
achieved through a substantial reduction in unsafe arrivals while imposing
deadlock and exposure costs. A paired ORCA-responsive pedestrian study tests
whether these effects persist when the crowd dynamics are changed.

**Engineering contribution, once the boundary question is settled:** a
progress-aware or recovery-aware TTC layer that preserves the rescues while
reducing deadlock and safe→unsafe degradation. Whether it should target a
*general* hard-braking problem or an *SFM-specific* exposure problem is exactly
what the ORCA-responsive pedestrian study decides.

Nearest prior work — Jafari, Tsai & Liu, arXiv:2607.09192 (10 Jul 2026),
National Cheng Kung University:

| | Theirs | Ours |
|---|---|---|
| Safety mechanism | TSFM replaces the distance dependency in the repulsion law with projected TTC | Separate speed-scaling filter applied after the force update |
| Route | Desired force, dynamically determined direction; no waypoint sequence described | Sparse A\* waypoint sequence |
| Pedestrians | "indoor and single pedestrian trials in facing encounter scenarios" | 48-agent crowd |
| Validation | Real nonholonomic robot, volunteers, Likert | Simulation |
| Question | Does PTTC/prediction improve safety? | How do guidance and safety mechanisms interact? |

Also cite: arXiv:2607.17604 (stability/comfort, same group); COSFM quadruped
paper (Biomimetics 2026, collision prediction + social norms).

**Do not claim** the A\*–SFM combination is novel, that TTC-plus-distance is a
discovery, or anything about pedestrian comfort.

## 7. Open — must resolve before writing

| Item | Why it blocks |
|---|---|
| Trap negatives | Only sensitivity measured. If successful trials also traverse flagged regions, discrimination collapses. |
| Split by map, not trajectory point | Otherwise the same basins appear in both development and test. |
| Grid convergence for area statistics | 0.1 m still misplaced equilibria by ~500×. Flagged-area percentages are grid-dependent until convergence is shown. |
| Incremental value over geometry | Obstacle clearance already gives 10× lift. If the static field is a transformed clearance map, novelty is limited. |
| Deadlock cause classification | Static / pedestrian-induced / controller-induced / other. The static field may only predict the first. |
| Interaction at proper n | Holm-adjusted p = 0.098 on development seeds. |
| No arm clears the 85% safe-success gate | Best is 60%. Outcome D — stop stacking heuristics, benchmark against ORCA. |
| ORCA baseline | Must use reference RVO2, not a hand-rolled MATLAB version. Pushes toward a Python port. |
| External pedestrian validation | The July 2026 papers have real-robot and human-participant data; simulation-only is materially weaker than it was. |
