# runs/ — reproducibility records

Each experiment script draws a fresh random seed per trial (`rng('shuffle')`),
applies it (`rng(seed)`), then runs the trial. Because every random element —
obstacle polygons, pedestrian start positions, velocities, and goals — is drawn
from MATLAB's global generator, **the integer seed fully determines the trial**:
re-running `rng(seed)` then the trial regenerates the exact same environment.

These small per-trial seed logs are committed here so a run can be reproduced
exactly. The bulky `.mat`/`.csv` outputs they correspond to are not tracked
(see `.gitignore`).

| File | Written by | Columns |
|---|---|---|
| `success_rate_seeds.csv` | `Success_Rate.m` | `trial, seed, SFM_success, AStarSFM_success` |
| `path_distance_seeds.csv` | `CSVTotalDistanceVsNumberTrials.m` | `trial, seed, AStarSFM_success, AStarSFM_dist, SFM_success, SFM_dist` |

## Reproducing a single trial

```matlab
% Read the seed for, say, trial 7 from the log, then:
rng(7th_seed_value);
Success();                 % success-rate trials
% or
GetTotalDistanceVsNumberTrialsVariables;   % path-distance trials
```
