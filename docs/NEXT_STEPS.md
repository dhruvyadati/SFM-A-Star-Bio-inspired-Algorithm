# Next steps — resumable work log

**Updated 30 August 2026 — Amendment G2 calibration returned a NEGATIVE result.**

## Current state, top of stack

| Amendment | State |
|---|---|
| **F** | negative calibration result; evaluation never launched, feasible candidate set empty |
| **G** | **RETIRED** — construct-validity failure. Pool at `v2/runs/v2/RETIRED_export_gblock_eval/`, must not be repurposed |
| **G2** | built, validated, calibrated. **NEGATIVE calibration result. Evaluation pool NOT launched.** |

**G2 outcome (see the appendix of `AMENDMENT_G2_DRAFT.md`, and
`v2/runs/v2/g2_calib_output.txt`):**

- Manipulation check **200/200**, re-derived from stored geometry independently
  of the generator. The construct is sound.
- Arm A deadlocks in **46/50** scenarios — constructed path invalidation does
  what retired G's harder geometry could not (0 deadlocks in 75).
- **Safe goal completion is 0.000 for every arm and all 18 configurations**
  (0 of 950 runs). The prespecified primary comparison cannot be made.
- No configuration passes the +0.05 safety screen.
- Replanning trades deadlock for contact: Δdeadlock −0.14..−0.26,
  Δunsafe +0.14..+0.26, Δsafe +0.000.

**Two amendments in a row have now returned negative calibration results
without reaching evaluation.** That is the honest state of the recovery layer
and should be reported as such, not engineered around.

**G2 is CLOSED.** Preserved as a negative targeted challenge result. Neither
evaluated nor redesigned.

### Do NOT do any of these
- relax the +0.05 screen, or regenerate the G2 pool to produce safe completions;
- pick a configuration out of the G2 descriptive table;
- launch the 150-scenario G2 evaluation pool;
- **create an Amendment G3**, TTC-enabled or otherwise. Experimentation on the
  recovery layer is finished.

### Claim boundaries — the TTC confound
G2 disables TTC to isolate planner recovery, so 0% safe completion cannot be
attributed to the recovery logic, to the missing collision filter, or to their
interaction. **Forbidden:** "replanning fails", "replanning worsens safety",
"recovery cannot resolve route invalidation", "the full proposed architecture
cannot safely recover from path invalidation". **Permitted:** *under the
TTC-disabled G2 challenge conditions, replanning exchanged immobilization for
contact without producing safe completion.*

G2 is a **targeted negative stress-test response** to reviewer 1.3 — not an
unqualified answer, since the evaluation pool was never touched and all
evidence is calibration-stage.

---

## Remaining work — these four items, then stop experimenting

1. **A\* + periodic replanning baseline.**
2. **Planning-component ablation.**
3. **Focused sensitivity sweeps.**
4. **Manuscript / Related Work / response letter.**

No further experiment families beyond these.

### Housekeeping
- `PAPER_LOG.md` predates G2 and does not mention it; fold the G2 outcome in
  when the manuscript work starts.
- `resubmission/*.md` remains the canonical editing source for the amendments.
  `docs/` inside the code repo is the archived snapshot committed for
  provenance; `build_review_bundle.sh` still builds from `resubmission/`.

---



---

## Mentor's 11 required actions, in order

| # | Action | Status |
|---|---|---|
| 1 | Fix backward progress across passed waypoints | **DONE** |
| 2 | Add missing recovery / path / TTC tests to the repo | **DONE** — 39 tests, 4 suites |
| 3 | Fix `Test_AnalyzeDose.py`; prove its negative control fails | **DONE** |
| 4 | Version frozen historical TTC source separately | **DONE** |
| 5 | Make `verify_confirm.py` + manifest pass without rewriting history | **DONE** |
| 6 | Integrate `PathProgress` / `ttc_brake_ex` / `ProgressAwareRelease` into `run_trial` | **DONE** |
| 7 | Fix the 2.99 s window | **DONE** |
| 8 | End-to-end inertness / emergency-precedence / replay / accounting tests | **DONE** — 7 pass, found the cap defect |
| 9 | Update Amendment F to the actual implementation | **DONE** |
| 10 | Archive revised code, tests, protocol, manifests | **DONE** — `review_status.json` |
| 11 | Amendment F | **DONE — negative calibration result; evaluation not launched, feasible candidate set empty** |

---

## Completed, with evidence

**1. `PathProgress` backward-motion defect — CONFIRMED REAL, FIXED.**
Reproduced the mentor's exact case: after passing a waypoint, `remaining`
saturated at the waypoint instead of rising with backward travel (0.2 gave
1.0 where 1.8 was correct). Cause: a strictly monotone segment index blocked
self-intersection jumps but discarded real reversal. Replaced with a
**plausibility bound** — a projection is admissible only if the implied arc
change is consistent with the robot's actual displacement this step. Backward
travel is admitted because displacement supports it; a self-intersection jump
is rejected because it would need a large arc change from a small step.
`py/test_path_progress.py`, **12 tests**, includes the mentor's exact case.

*My original test suite tested backward motion WITHIN a segment, never ACROSS a
passed waypoint. That is why it passed while the code was wrong.*

**3. `Test_AnalyzeDose.py`.** Confirmed failing. Cause: the synthetic generator
emitted `ttcStop=0.8` and `ttcMinGap=0.4` separately, but the real generator
runs the three sweeps' shared centre ONCE under `ttcHorizon=2.0`. The test
passed when written, then the centre fix landed in `analyze_dose.py` and **the
test was never re-run**, so the "passes" claim went stale. Now mirrors the real
labelling: 7/7 pass.

**4 + 5. Source provenance.** `archive/confirm_code/py/` now holds the exact
sources that ran the confirmation: `ttc.py` = `b89dcad9104b6db9` and
`verify_confirm.py` = `026ef1c6e8b18b51`, both verified against the frozen
record. `verify_confirm.py` checks the archived copies, not the development
head. **The manifest was NOT rewritten** — it is correct about what ran.
`py/test_ttc_result_compatibility.py` proves the current head is
behaviour-identical over 4000 archived randomised states, with the seed and
generator in the file so the claim is reproducible.
**GATE PASSED.**

**7. Window drift.** `int(3.0/0.01)` = 300 samples spans 2.99 s. Now
`int(round(window/dt)) + 1`, and the constructor raises if any duration is not
an exact multiple of `dt`.

---

## Outstanding — the only things left

**Five `TBD_` decisions in Amendment F**, each needing a written engineering
justification and none of which may be chosen after seeing any outcome:

| Symbol | What it fixes | Constraint |
|---|---|---|
| `TBD_CAL_TOL` | calibration unsafe-arrival tolerance | NOT the evaluation margin |
| `TBD_MARGIN` | Gate 1 non-inferiority on unsafe arrival | one endpoint, one margin |
| `TBD_RESCUE` | Gate 2 paired rescue-rate floor on pool `U` | `U` sized prospectively |
| `TBD_DEADLOCK` | Gate 3 minimum worthwhile deadlock reduction | must justify a new control layer |
| power / N | evaluation sample size | from calibration discordance rates |

**Then:** export the F evaluation set (disjoint seed family, same four-condition
balance), archive, and only then run the two reference arms plus 18 Stage A
configurations.

**Second review pass** recommended before the sweep — the previous one found a
defect that would have biased the calibration.

**Amendment G** was built, then **RETIRED** as a construct-validity failure,
and superseded by **G2**, which reached calibration and returned a negative
result. See the status block at the top of this file.

---

## State that must not be lost

- **F calibration set exists and is verified**: 160 scenarios, 40/condition,
  0 rejected, collective sha256 `1dc1b1d2928f961f...`, manifest at
  `v2/runs/v2/fcal_manifest.json`. Geometry-level disjoint from
  `export_confirm` (160/160 unique maps and layouts).
- **MATLAB is at** `/Applications/MATLAB_R2025b.app/bin/matlab` (not on PATH).
  Export needs BOTH `v2/` and the parent `.m` files — `MakeScenario` calls
  `GenerateEnvironmentGraphObject` from the parent.
- **Stage A screening defaults** (labelled defaults, NOT selections): release
  0.60, max recovery 3.0 s, hysteresis 1.0 s, cooldown 2.0 s, episode cap 2.
- **Stage A grid**: window {2,3,4} s x progress {0.10,0.20,0.30} m x activation
  {0.6,0.8} = 18 configurations.
- **Cooldown semantics**: re-arm requires 2.0 s of *demonstrated restored
  progress*, latched once earned. Not elapsed time — a timer lets a permanently
  stalled robot manufacture repeated episodes.
- **Amendment G2** pools are frozen and geometrically validated: 50 calibration
  + 150 evaluation, manifest `v2/runs/v2/g2_manifest.json`, manipulation check
  200/200. The **evaluation pool has never been run on** and must stay that way
  unless a newly frozen design justifies it.
- Reviewer 1.3 now has an answer, and it is a negative one: under constructed
  path invalidation the baseline deadlocks (92%), and the recovery layer
  converts deadlock into contact rather than into safe arrival.
