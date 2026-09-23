# ORCA robot comparison

Status: all 3,000 development and 1,000 evaluation trials completed successfully.
The selected buffer is 0.02 m. See `aligned/analysis.json` and `aligned/RESULTS.md`
for the validated paired comparisons.

Use the existing robot ORCA controller with SFM pedestrians. Evaluate guided and
direct navigation separately. Use all 300 development scenarios at margins
0, 0.02, 0.05, 0.10 and 0.20 m (3,000 trials), then both planners on all 500
existing evaluation scenarios at one selected margin (1,000 trials). Other ORCA
parameters remain fixed. This is a margin screen, not exhaustive ORCA tuning.

Selection preserves the existing executable rule: among guided configurations
within 5 percentage points of the highest safe-success rate, prefer zero total
contacts, then the smallest margin. If none has zero contacts, pick the smallest
eligible margin, not minimum nonzero severity. The direct arm uses the same
selected margin. No selection on evaluation results is allowed.

The three previously inspected evaluation scenarios are recorded in the runner.
The existing evaluation set is not newly untouched evidence. Prior inspection
must remain disclosed in any interpretation. New outcomes are supplementary.

The runner enables aligned stepping: both controllers use the shared pre-update
state, pedestrian positions advance with their updated capped SFM velocities,
contacts use the pre-update force routine's gaps, and waypoint capture occurs
after integration. The robot still chooses its velocity through ORCA. True
radii and strict negative-gap contact scoring match the existing SFM results.
Neither controller implements continuous swept collision detection. Historical
ORCA calls retain their default behavior; the aligned option is explicit.

Two preliminary development trials used the earlier timing. They remain in
`development/` and are excluded from the aligned run. The new run starts in
`aligned/development/` with a distinct source manifest.

Each output directory has an exclusive process lock, a source/runtime/scenario
hash manifest, atomic per-trial JSON records, and a completion-labelled summary.
Resume refuses source, runtime or input changes. A complete development screen
is required to freeze selection. Evaluation validates the selection and its
referenced development result files before running. Failed trials raise errors
and are not silently counted as completed observations.

Commands from the repository root (output paths are outside it):

```sh
python py/run_orca_experiment.py development --output ../experiments/orca_robot/aligned/development --workers 4
python py/run_orca_experiment.py freeze --output ../experiments/orca_robot/aligned/development
python py/run_orca_experiment.py evaluation --output ../experiments/orca_robot/aligned/evaluation --selection ../experiments/orca_robot/aligned/development/selection.json --workers 4
```

For a bounded execution use `--max-new-trials 4`; repeat without that flag to
resume the full stage. Do not edit source files while a stage is running.

## Analysis

Run `python experiments/orca_robot/analyze_results.py` from the parent of the
repository. It validates recorded source files, scenario hashes, all trial keys,
selection hashes and the unchanged original result file before computing
descriptive paired intervals. Add `--strict-runtime` to require the exact
recorded Python, NumPy and RVO2 binary as well. Analysis alone does not execute
RVO2. The analysis plan is post-evaluation; the original primary analysis is
unchanged. A supplementary exclusion removes the three previously inspected
scenarios.
