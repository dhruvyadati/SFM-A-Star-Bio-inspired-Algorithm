# Fixed parameter screen

The protocol fixes 100 development scenarios and seven settings for each of two
guided controllers, for 1,400 trials. `manifest.json` records source, scenario,
protocol and runner hashes. `trials/` contains one atomic result file per job.
`EXECUTION.md` records the interruption and resumption without changing settings.

Use the exact recorded Python runtime and NumPy version for simulation replay;
the runner compares the complete Python version string in its saved manifest. From any
directory, `python run.py` verifies the saved manifest, retains completed trials,
and schedules only missing records. It does not retry recorded failures. To
rerun completed trials, make a separate copy of this experiment directory and
remove only that copy's generated trial files; preserve its original manifest
and sources. Never overwrite the archived results in place.

To reconstruct results without simulation, run:

```sh
python experiments/parameter_sensitivity/analyze.py
```

from the repository root. NumPy is required. The analyzer refuses an
incomplete result set, recorded failures, changed simulation sources, changed
scenario inputs, duplicate job keys or missing condition/arm combinations.
`analysis.json` and `RESULTS.md` report full outcome counts, paired TTC
safe-completion differences, and changes relative to nominal. Condition-level
counts are in `condition_analysis.json`, with 25 matched scenarios per condition.

The results are descriptive and exploratory. These are one-at-a-time
perturbations, not parameter tuning, empirical human-speed calibration or a test
of parameter interactions. The original confirmatory analysis is unchanged.

The active manifest records the relocated runner hash. Original execution
manifest and runner bytes are preserved under `../provenance/parameter_sensitivity/`.
