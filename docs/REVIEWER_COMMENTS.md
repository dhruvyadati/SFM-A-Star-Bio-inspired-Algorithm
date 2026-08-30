# Access-2026-31192 — Reviewer Comment Tracker

Every distinct comment from both reviewers, with the phase that resolves it and
current status. This is the backbone of the "Response to Reviewers" document
IEEE Access requires on resubmission (comment → response → action taken).

**Status key:** `DONE` verified in v2 code · `PARTIAL` started, gap noted ·
`TODO` not begun · `WRITE` manuscript text only, no new experiment

Phases are defined in the resubmission plan: 0 de-risk · 1 correctness/protocol ·
2 baselines · 3 metrics · 4 analyses · 5 trap rebuild · 6 writing.

---

## Reviewer 1

| # | Comment | Phase | Status | Notes |
|---|---------|-------|--------|-------|
| 1.1 | Compare against prior A*–SFM, waypoint-guided SFM, global–local, socially aware methods; add table (planning strategy, replanning, dynamic handling, cost, validation env) | 6 | TODO | Needs lit survey first |
| 1.2 | "Preserves global optimality" is wrong — SFM modifies the executed trajectory | 6 | WRITE | Reword to "globally-informed"; optimality holds only for the static grid plan |
| 1.3 | One-shot A* with no replanning: prolonged blockage, inaccessible waypoints, large displacement | 1,4 | **TODO — UNRESOLVED, publication risk** | PROTOCOL §5a specified exactly this fallback but a 3 Aug 2026 audit found it was **never implemented**. All studies ran strict one-shot A*. Amendment F does NOT answer this: F releases the TTC brake, not the route. Needs either (a) an honest limitation + future-work statement, or (b) a separate small §5a three-way ablation with its own protocol. Also touches 1.37. |
| 1.4 | Eq (5) uses raw displacement; needs justification + dimensional analysis | 1 | **DONE** | `ComputeForces.m` §1 uses unit vector; units documented |
| 1.5 | Eqs (6),(7) unnormalized; plot force vs distance | 1,6 | **DONE** (code) | Monotonicity asserted in `Test_ForceSanity.m`; figure still to draw |
| 1.6 | Eq (9) is not semi-implicit Euler | 1 | **DONE** | `RunTrial.m` advances v then x |
| 1.7 | Point agent vs one-cell margin; state cell size, robot/ped radius, continuous collision checking | 1 | PARTIAL | Units + radii in `SfmParams.m`; swept-segment collision check still TODO |
| 1.8 | v_min = 1 prevents safe stopping | 1 | **DONE** | vMin removed; `Test_ForceSanity.m` check 6 proves an agent rests at goal |
| 1.9 | Same equations for pedestrians and robots; how are ped goals updated, do peds avoid obstacles/each other | 1 | PARTIAL | Goal resampling added; realism vs real crowd data not yet argued |
| 1.10 | Add ORCA, DWA, velocity-obstacle, local-planner A*, or a published hybrid | 2 | TODO | **Largest single item** |
| 1.11 | A*-only with periodic replanning is a valid baseline | 2 | TODO | |
| 1.12 | Two robots share one simulation and interact | 1 | **DONE** | `MakeScenario`/`RunTrial` run arms separately from a shared seed |
| 1.13 | Justify 2-unit success threshold; report sensitivity | 4 | PARTIAL | `P.rGoal` is now a swept parameter; sweep not yet run |
| 1.14 | Termination undefined; distinguish deadlock/collision/oscillation/timeout | 1 | **DONE** | `RunTrial.m` classifies outcome; maxSteps justified in `SfmParams.m` |
| 1.15 | No collision rate, min distance, near-collision, margin violations | 3 | **DONE** | Instrumented in `RunTrial.m` |
| 1.16 | No social metrics: zone intrusion, disturbance, passing side, smoothness, accel, jerk | 3 | PARTIAL | Zone time, accel, jerk done; passing-side and disturbance TODO |
| 1.17 | No execution times: A* time, SFM update time, per-trial cost, control frequency | 3 | **DONE** | Graph build and A* query timed separately; A* ≈ 11 ms, build ≈ 0.85 s |
| 1.18 | Trap analysis uses different repulsion parameters than the navigation runs | 5 | TODO | Recompute with navigation parameters |
| 1.19 | δ = 0.3 unjustified; sweep threshold | 5 | TODO | |
| 1.20 | Force field excludes moving pedestrians though the study is about dynamic crowds | 5 | TODO | Recompute with pedestrians; test basin persistence |
| 1.21 | Average gradient 248.0 has no units or reference scale | 5 | TODO | Now expressible in m/s²/m under the declared units |
| 1.22 | Force cost along path: interpretation unclear; verify it predicts difficulty | 5 | TODO | Correlate against outcome per trial |
| 1.23 | Path length on successful trials only → selection bias | 1 | **DONE** | Final distance-to-goal over ALL trials added to `Run_Phase0.m` |
| 1.24 | Clarify pairing; paired tests more appropriate | 1 | **DONE** | `StatsLib.signedRank` + exact McNemar on shared seeds |
| 1.25 | CI method not stated | 3 | **DONE** | Wilson score for proportions, percentile bootstrap for means |
| 1.26 | Add effect sizes; address multiple comparisons | 3 | **DONE** | Rank-biserial, Cliff's δ, Cohen's h, Holm–Bonferroni in `StatsLib.m` |
| 1.27 | Vertex range 3–10 in methods vs 3–20 in Fig. 4 | 6 | WRITE | Two different generators; must be stated explicitly |
| 1.28 | Workspace units not mapped to real dimensions | 1 | **DONE** | 1 cell ≡ 1 m declared in `SfmParams.m`; 50 × 50 m plaza |
| 1.29 | Obstacle-area units ambiguous | 6 | WRITE | m², shoelace, pre-rasterisation |
| 1.30 | Report how often maps were rejected for having no A* path | 1 | **DONE** | Counted and printed by `Run_Phase0.m` |
| 1.31 | "immune", "unaffected", "decoupled" too strong | 6 | WRITE | |
| 1.32 | Need trajectory evidence of SFM trapped vs hybrid escaping the same region | 5,6 | TODO | Paired-seed trajectory overlay figure |
| 1.33 | Figures need trajectories, trap regions, waypoints, clearance margins | 6 | TODO | |
| 1.34 | Parameter sensitivity: τ, a, a_s, radii, waypoint threshold, dt, velocity limits, δ | 4 | TODO | |
| 1.35 | Crowd density fixed at 48; vary density and pedestrian speed | 4 | TODO | `numPedestrians`, `v0Ped` already parameterised |
| 1.36 | Add crossing flows, opposing streams, bottlenecks, narrow passages, groups, sudden obstruction | 4 | TODO | Needs a scenario generator beyond random polygons |
| 1.37 | Ablation: one-shot A*, collinear pruning, waypoint thresholding, SFM control | 4 | TODO | |
| 1.38 | Describe pruning formally; can it push segments too close to obstacles? | 1,6 | TODO | `GraphFast.dmin` gives the clearance test needed |
| 1.39 | One-cell margin may not guarantee diagonal / continuous-deviation clearance | 1 | PARTIAL | `dmin` exported; formal analysis still to write |
| 1.40 | Validate in ROS/Gazebo, CrowdNav, or SocNavBench | — | **IN PROGRESS** | Path A chosen. CrowdNav feasibility spike PASSED all four frozen conditions (Amendment H). Controller wraps as a `Policy` returning `ActionXY` with no architectural change. Bounded transfer check to run: guided ± TTC, frozen params, 6 outcomes. NOT satisfied by the Python port, which tests implementation reproducibility only. |
| 1.41 | Expand reproducibility: seeds, code, parameter tables, generation constraints, ped rules, trial counts | 1 | PARTIAL | Seeds logged; `StatsLib` removes the paid-toolbox dependency |
| 1.42 | Literature: APF local minima, equilibrium detection, deadlock recovery, socially aware planning, benchmarks | 6 | TODO | |
| 1.43 | Acknowledge limitations: static map, one-shot, point agent, ideal localisation, no sensor noise, simple pedestrians, no hardware | 6 | WRITE | |
| 1.44 | Conclusion overstates generality | 6 | WRITE | |
| 1.45 | Language editing; "optimal", "immune", "human-like", "trap basin", "real-time" | 6 | WRITE | |

## Reviewer 2

| # | Comment | Phase | Status | Notes |
|---|---------|-------|--------|-------|
| 2.1 | Direct combination of existing methods; one-shot planning and pruning are implementation choices | 6 | TODO | Same response as 1.1; reposition contribution around the diagnostic |
| 2.2 | Repulsion ∝ d·exp(−d) vanishes at contact, contradicting the claimed barrier | 1 | **DONE** | Corrected; verified maximal at contact |
| 2.3 | Forces and accelerations combined without specifying mass or units | 1 | **DONE** | Unit mass stated explicitly in `ComputeForces.m` header |
| 2.4 | Eq (9) not semi-implicit Euler and inconsistent with Eq (10) | 1 | **DONE** | |
| 2.5 | Trap analysis uses a static environment and different parameters, so Table 1 does not describe the simulated system | 5 | TODO | Same as 1.18/1.20 |
| 2.6 | 3–10 vs 3–20 vertices; may obstacles overlap; is a feasible path guaranteed; how are no-path trials handled | 1,6 | PARTIAL | Infeasible maps now counted; overlap policy still to state |
| 2.7 | v_min prevents stopping, yet success latches on a single instant in the goal region | 1 | **DONE** | Dwell requirement (0.5 s) replaces the latch |

---

## Contested — argue rather than comply

**1.40 (Gazebo / CrowdNav / SocNavBench).** Full ROS/Gazebo validation is a
separate paper's worth of work and the reviewer's framing ("MATLAB-only is
insufficient to support broad deployment claims") can be met by removing the
deployment claims instead. Proposed response: moderate every deployment claim,
state the simulation-only scope in the abstract and conclusion, and add one
recognised benchmark scenario set to show the result is not an artefact of the
random-polygon generator. Note that this is the single highest-risk item — if
the editor treats it as mandatory, it must be done or the paper goes elsewhere.

---

## Additional defects found in the code that no reviewer caught

Fixing these unprompted strengthens the response letter considerably.

| # | Defect | Location | Status |
|---|--------|----------|--------|
| X.1 | Obstacle penetration was self-reinforcing: `p_poly_dist` returns a negative distance inside a polygon, which v1 fed into both the cutoff test and the direction vector, driving a penetrating agent deeper in | `Force_O_Stat.m:20-23` | **DONE** |
| X.2 | Repulsion was active only within `r_inf = 1`, the exact range over which `d·exp(−d)` is *increasing* — so it was weakest at contact and strongest at the cutoff | `Force_O_Dyn.m`, `Force_O_Stat.m` | **DONE** |
| X.3 | Pedestrians were never given new goals, and could not stop, so the crowd degenerated into 48 agents jittering around fixed points | `SuccessParam.m:57-67` | **DONE** |
| X.4 | Reported A* cost was dominated by graph construction (~21 s), not search; `Graph.m` made ~225k point-to-polygon calls and `AStar.m` called `findedge` per neighbour | `Graph.m`, `AStar.m` | **DONE** — 26× faster, output proven identical |
| X.5 | `StatisticalTests.m` depends on the paid Statistics Toolbox (`ranksum`), so the published analysis is not reproducible on base MATLAB | `StatisticalTests.m` | **DONE** — `StatsLib.m` is toolbox-free |
| X.6 | The 6000-step horizon was only ~1.1× the free-space traverse time, so timeouts partly measured the clock rather than entrapment | `SuccessParam.m:174` | **DONE** — raised to 15000 with justification |
