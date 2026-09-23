# Robot navigation with A*, social forces and predictive braking

This repository contains navigation simulators, fixed scenarios, saved outcomes
and analysis utilities. The current implementation combines a static A* route
with a social-force controller and an optional time-to-collision speed filter.
An ORCA robot controller provides an additional comparison among social-force
pedestrians, with direct-goal and A*-guided variants.

## Current results

The fixed factorial evaluation uses 500 matched scenarios and four controllers
(2,000 trials). With A* guidance, safe completions were 176/500 without filtering
and 215/500 with filtering: a gain of 7.8 percentage points. Arrivals with contact
fell by 19.8 points, while deadlock rose by 12.0 points. The confidence interval
for the safe-completion gain spans the prespecified 7.5-point meaningful-effect
threshold. Without guidance, the safe-completion difference was -0.4 points.

The supplementary ORCA comparison used 3,000 development trials to select a
0.02 m margin and then 1,000 evaluation trials. Guided ORCA completed 426/500
scenarios safely (85.2%); direct-goal ORCA completed 38/500 (7.6%). The evaluation
reuses the factorial scenarios and is descriptive rather than an independent
confirmatory study. The controllers have different velocity dynamics, including
final-goal versus active-waypoint speed tapering.

Here, “safe” means arrival satisfying the stopping rule with no recorded contact.
Contact is sampled at integration states, not continuously between steps; final
updates can be unobserved. These simulations do not establish real-world safety,
human-like behavior or continuous collision freedom.

## Layout

| Location | Contents |
|---|---|
| `py/` | Current simulation, analysis, verification and regression utilities |
| `v2/` | MATLAB implementation and scenario exporters |
| `v2/runs/v2/` | Saved results, manifests and scenario pools, when distributed with data |
| `archive/confirm_code/` | Exact archived sources used for the fixed factorial results |
| `docs/` | Protocols, environment details and historical development records |
| Root-level MATLAB scripts | Legacy implementation and exploratory utilities |
| `runs/` | Historical seed records |

Large data files may be supplied separately from a Git checkout. Preserve their
relative paths when extracting the data distribution. Saved scenarios are the
reference inputs; seeds alone do not ensure cross-version or cross-platform
identity.

## Reproducing saved analyses

From the repository root, with the saved data present:

```sh
python -m pip install -r requirements.txt
python py/verify_confirm.py
python py/analyze_confirm.py
python py/verify_fcal.py --fast
```

The verification gates check recorded inputs and source hashes before analysis.
Do not change frozen sources or rewrite their manifests to silence a mismatch.
A saved result establishes what was executed; it does not validate every earlier
interpretation of that result.

A focused regression check is:

```sh
python -m pytest py/test_path_progress.py py/test_recovery_state_machine.py py/test_ttc_result_compatibility.py py/test_orca_alignment.py py/test_orca_experiment.py -q
```

ORCA simulation and its alignment tests additionally require the `rvo2` Python
extension built from Python-RVO2. Consult `docs/ENVIRONMENT.md` and the distributed
runtime records for the recorded versions. Plotting requires Matplotlib. MATLAB
is needed to regenerate MATLAB scenarios, not to analyze the saved Python data.
Full simulation replay tests take substantially longer than the focused checks.

## Legacy results

The root-level MATLAB scripts are retained for historical inspection. They use
an earlier implementation and outcome definition. Their previous path-length,
trap-region and collision-free claims are not supported by the corrected
analyses and must not be treated as current results. In particular, the earlier
36% path-length reduction and force-field trap-point statistics are withdrawn.
The old simultaneous-two-robot setup is not the matched, separately executed
comparison used by the current simulator.

The third-party `p_poly_dist.m` helper has no verified redistribution license
and is excluded from the distributable snapshot. Historical MATLAB execution
may require obtaining it separately. Current saved-result Python analysis does
not require this helper.

## License

Original software is licensed under the MIT License; see `LICENSE`.
Original simulation data and documentation are licensed under CC BY 4.0; see
`DATA_LICENSE.md`. Existing third-party licenses and notices remain applicable.

## Additional experiment results

See [experiments/README.md](experiments/README.md) for saved ORCA comparisons,
parameter sensitivity, clearance diagnostics, trajectory replays, and commands
to verify file checksums and regenerate analyses without running simulations.
