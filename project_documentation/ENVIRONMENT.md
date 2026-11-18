# Environment Record

Pinned for reproducibility. The cross-language validation is only meaningful
against a stated toolchain.

| Component | Version |
|---|---|
| Platform | macOS-26.5.1-x86_64-i386-64bit-Mach-O |
| MATLAB | 25.2.0.3177638 (R2025b) Update 5 |
| MATLAB toolboxes | base + Image Processing + Mapping. **No** Statistics/ML Toolbox — all statistics are implemented in `v2/StatsLib.m` |
| Python | 3.13.2 |
| NumPy | 2.3.1 |
| Cython | 3.2.9 |
| CMake | 4.4.0 |
| Compiler | Apple clang version 11.0.3 (clang-1103.0.32.62) |
| RVO2 bindings | sybrenstuvel/Python-RVO2 @ `c2c46ba` |

## RVO2 build notes

Two issues anyone rebuilding will hit:

1. **CMake 4.x removed compatibility with CMake < 3.5**, which RVO2's
   `CMakeLists.txt` still declares. Pass `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`
   through `setup.py`.
2. **`MACOSX_DEPLOYMENT_TARGET` must match the Python build, not the OS.**
   Conda's Python 3.13 here is `macosx-10.15-x86_64`; setting 14.0 fails with
   "invalid version number".

## Cross-language validation

Six canonical fixtures (`v2/runs/v2/export/canonical_*.json`) are permanent
repository artifacts, not scratch files.

Tolerances were specified BEFORE the comparison was run and are recorded in
`py/compare_engines.py` unchanged, even though observed error was far smaller:

| Quantity | Tolerance | Observed |
|---|---|---|
| outcome | exact | exact, 6/6 |
| steps | 1% | exact, 6/6 |
| path length | 0.1% | identical to 3 dp |
| final distance | 1e-3 m | below |
| trajectory max error | 0.05 m | 8.9e-14 m |
| trajectory RMS error | 0.01 m | 4.2e-14 m |

Statement of what this supports: the independently implemented engines
produced trajectory agreement at numerical roundoff across canonical cases,
giving strong end-to-end evidence that the corrected update order,
integration, force evaluation, clamping, goal taper, obstacle handling and
deterministic goal-pool logic were reproduced consistently. It is end-to-end
evidence, not an independent proof of each detail separately.

## Bugs found via the canonical fixtures

Reproducibility changelog, not paper results:

1. Zero-pedestrian scenarios crashed MATLAB `RunTrial` — `min([])` returns
   empty. Generated trials always carry a crowd, so this never surfaced.
2. MATLAB `jsondecode` returns a COLUMN vector for a flat JSON array, so a
   single agent's `[x, y]` arrives 2x1 and an unconditional transpose swaps
   the dimension meaning. Python's loader was already correct.
3. An unset optional scalar (`aPedRobot = []`) became an empty tuple in the
   Python loader before the null check, breaking a broadcast at use time.


## SCENARIO GENERATION — blocker PARTIALLY RESOLVED (3 August 2026)

> **UPDATE.** MATLAB R2025b Update 5 is installed locally at
> `/Applications/MATLAB_R2025b.app/bin/matlab` (not on `PATH`, which is why
> `which matlab` reported it missing). The **F calibration set is now
> exported and verified**. The remaining sets (F evaluation, G calibration,
> G evaluation, optional G benchmark) are still outstanding.

### How the export was run

Executed from a clean directory (`sra/matlab_export/`) containing only copied
`.m` sources, to confirm the procedure works standalone rather than relying on
anything in the working tree:

```
cp SFM-A-Star-Bio-inspired-Algorithm/v2/*.m  matlab_export/v2/
cp SFM-A-Star-Bio-inspired-Algorithm/*.m     matlab_export/
cd matlab_export/v2
/Applications/MATLAB_R2025b.app/bin/matlab -batch "Export_FCalibration"
```

**The parent-directory `.m` files are required.** `MakeScenario` calls
`GenerateEnvironmentGraphObject`, which lives one level above `v2/`. Copying
only `v2/` fails with `Unrecognized function or variable`. No toolboxes are
needed.

### F calibration set — verified

160 scenarios, 40 per condition, **0 rejected in 160 attempts**.
Collective sha256 `1dc1b1d2928f961f...`; manifest at
`v2/runs/v2/fcal_manifest.json`.

| Check | Result |
|---|---|
| counts, 40 per condition | PASS |
| no overlap with 800 prior seeds | PASS |
| new seeds all >= 2^31, prior all < 2^31 | PASS |
| every scenario loads with waypoints and pedestrians | PASS |
| hybrid / hybrid-TTC reproduce exactly on repeat | PASS |
| **160/160 distinct obstacle maps and crowd layouts** | PASS |
| **0 maps or layouts shared with `export_confirm`** | PASS |

The last two matter most: seed disjointness alone would not detect a generator
that ignored its seed. Geometry-level comparison confirms the seed genuinely
drives generation and that no scenario duplicates the confirmatory set.

### Original blocker record — SUPERSEDED, RETAINED FOR AUDIT ONLY

> **HISTORICAL ERROR, WITHDRAWN 3 August 2026.** Every line below is quoted
> because it is FALSE. MATLAB R2025b was installed at
> `/Applications/MATLAB_R2025b.app` throughout; the check used `which matlab`,
> which searches PATH only. The false claim reached Amendment E as
> justification for a design choice. No result was affected — the
> fixed-scenario design was independently correct.

>
> **The text below is FALSE and is retained only so the error is auditable.**
> It states that MATLAB and Octave are not installed. MATLAB R2025b Update 5 was
> installed at `/Applications/MATLAB_R2025b.app` throughout. The availability
> check used `which matlab`, which searches `PATH` only; the application is not
> on `PATH`. A single `ls /Applications/MATLAB*` would have found it.
>
> The false claim propagated into Amendment E as justification for a design
> choice, and delayed the calibration export across several exchanges. Both the
> amendment and this section are corrected; **no result was affected**, because
> the fixed-scenario design was independently correct.
>
> Do not read any sentence below as a current statement of the environment.
>
>
> Amendments F and G both require **new, disjoint scenario families**. No
> untouched set exists. Full inventory:
>
> | Set | n | Used for |
> |---|---|---|
> | `export` | 15 | canonical cross-language fixtures |
> | `export_confirm` | 500 | confirmation, dose-response, trajectory, ORCA study |
> | `export_dev` | 300 | margin screen, development factorial, ORCA robustness |
> | `export_rand` | 24 | randomised differential-testing fixtures |
>
> Reusing `export_confirm` for calibration would tune the recovery controller on
> the very cases whose TTC failures motivated it. Reusing it for evaluation would
> yield another paired follow-up, not an independent test. `export_dev` is worse:
> it already influenced architecture and parameter choices.
>
> MATLAB and Octave are **not installed in this environment**, so the frozen
> generator (`Export_ConfirmScenarios.m` / `MakeScenario.m`) cannot be run here.
>
> ### Acceptable routes, in order of preference
>
> 1. Run the frozen MATLAB generator on another machine or by a collaborator, and
>    export two immutable sets: a modest **recovery-calibration** set balanced
>    across the four conditions, and a larger separately seeded **evaluation**
>    set. G needs either a third set or a declared allocation from the evaluation
>    family.
> 2. A temporary or licensed remote MATLAB, used **only** to export scenarios.
> 3. Previously generated but genuinely untouched scenarios — **none exist**, per
>    the inventory above. "Unused outcomes" would not suffice anyway if the maps
>    were inspected during development.
> 4. Porting the generator to Python **only** as a separate, validated
>    infrastructure project, requiring deterministic comparison against
>    MATLAB-generated output before its scenarios could support a formal
>    evaluation.
>
> ### Why not simply port it now
>
> The cross-language validation establishes **engine** agreement on exported
> fixtures — 1e-14 on canonical cases, exact on real trial configs. It says
> nothing about a newly written **generator**. Porting it now would change
> scenario generation at the same time as the controller experiment, so a
> difference in results could not be attributed to either.
>
> Sample sizes are deliberately NOT proposed until the expected discordance rates
> and the non-inferiority margin are set.
>