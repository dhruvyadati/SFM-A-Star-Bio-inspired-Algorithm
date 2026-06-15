# Joint A* and Social Force Model for Autonomous Navigation

Autonomous navigation in crowded, dynamic spaces is hard: purely reactive planners like
the bio-inspired **Social Force Model (SFM)** move fluidly among pedestrians but stall in
local-minima deadlocks when competing obstacle forces cancel out, while global planners
like **A\*** produce optimal collision-free paths around static obstacles but cannot react
to moving pedestrians in real time.

This project introduces a lightweight hybrid that pairs a one-shot A\* global plan with
continuous SFM-based local control — no replanning and no training data required. It also
contributes a **force-field trap-point analysis** that maps the net-force magnitude field of
a dynamic environment, quantifies the fraction of the workspace that forms deadlock basins,
and measures the spatial gradient of force complexity, explaining *when and why* reactive-only
navigation fails.

Across 50 trials, roughly 20% of grid points were trap regions spread over an average of
8.5 basins — the structural failure mode of reactive-only navigation. By steering around
these traps with A\*-guided waypoints, the hybrid improves navigation success from **22% to
93%** while reducing average path length by **36%** relative to the SFM-only baseline.

The implementation and all experiments are written in MATLAB.

---

## Requirements

- MATLAB R2020a or later (no additional toolboxes required)

The statistical tests (two-proportion z-test, Wilcoxon rank-sum) are implemented directly
with base functions, so the Statistics and Machine Learning Toolbox is **not** needed.

---

## Repository Structure

| File | Purpose |
|---|---|
| `Success.m` | Single trial: runs A* + SFM and SFM-only in the same environment |
| `Success_Rate.m` | Sweeps N trials, reports success rates and a two-proportion z-test |
| `CSVTotalDistanceVsNumberTrials.m` | Sweeps N trials, reports path lengths and a Wilcoxon rank-sum test |
| `StatisticalTests.m` | Loads saved results and prints a significance summary |
| `AStar.m` | A* shortest-path search on the weighted graph |
| `Graph.m` | Constructs the 8-connected weighted grid graph |
| `Force_G.m` | SFM goal attraction force |
| `Force_O_Dyn.m` | SFM dynamic agent repulsion force |
| `Force_O_Stat.m` | SFM static obstacle repulsion force |
| `GenerateObstacle.m` | Random polygon obstacle generator |
| `GenerateEnvironmentGraphObject.m` | Environment setup for graph-based experiments |
| `GenerateEnvironmentNumberVertices.m` | Environment setup for vertex-count sweep |
| `GetNumberVerticesVsSuccessRate.m` | Success rate vs. obstacle vertex count |
| `GetTotalDistanceVsNumberTrialsVariables.m` | Per-trial distances and success flags |
| `removeColinearNodes.m` | Prunes collinear waypoints from the A* path |
| `PlotNetForceFromAgentPerspective.m` | Computes and visualizes the net force vector field |
| `RunForceFieldStats_NTrials.m` | Runs force-field trap-point analysis over N trials |
| `p_poly_dist.m` | Polygon-point distance utility (third-party) |
| `ptInObs.m` | Point-in-obstacle test |
| `Visual.m` | Animation of agent trajectories |
| `TimevsDistance.m` | Distance-from-goal vs. time step analysis |

### Environment and visualization notes

- Obstacles are drawn as red transparent polygons; the start point is marked in black
  and the goal point in green.
- The environment-generation utilities build an occupancy grid over the bounded region
  (`1 = free space`, `0 = obstacle`), sized `[yMax, xMax]`, which is used for A* pathfinding.

---

## Running the Experiments

### Success rate over 100 trials (two-proportion z-test)
```matlab
Success_Rate            % runs 100 trials; reports SFM and A*+SFM success rates + z-test
```

### Path-length comparison (Wilcoxon rank-sum test)
```matlab
CSVTotalDistanceVsNumberTrials   % runs 50 trials; reports mean distances + rank-sum test
```

### Significance summary
```matlab
StatisticalTests        % loads the saved .mat results and prints the report
```
Run `Success_Rate` and `CSVTotalDistanceVsNumberTrials` first — they write the
`.mat` files that `StatisticalTests` reads.

### Success rate vs. vertex / obstacle count / area
```matlab
GetNumberVerticesVsSuccessRate
% For obstacle-count and area sweeps, adjust the parameters in Success_Rate.m
```

### Distance efficiency over time
```matlab
TimevsDistance
```

### Force-field trap-point analysis
```matlab
RunForceFieldStats_NTrials          % runs 50 trials, prints trap-point / basin metrics
PlotNetForceFromAgentPerspective    % single-trial net-force visualization
```

---

## Key Parameters

| Parameter | Value | Description |
|---|---|---|
| `t_r` (τ) | 0.5 | Reaction time |
| `v_min` | 1 | Minimum speed |
| `v_max` | 4 | Maximum speed |
| `a` | 800 | Dynamic repulsion strength |
| `b` | 1 | Dynamic repulsion decay |
| `a_s` | 10000 | Static repulsion strength |
| `r_inf` | 1 | Dynamic influence radius |
| `r_inf_s` | 1 | Static influence radius |
| `dt` | 0.01 | Time step |
| `p` | 50 | Total agents (2 robots + 48 pedestrians) |
| `numObstacles` | 10 | Number of obstacles |
| `maxArea` | 150 | Maximum obstacle area (sq. grid units) |

> Confirm these against `Success.m` before relying on them — they should agree with the
> values set in the source.

---

## Simulation Design Notes

- The **start position** is fixed at (1, 1) and the **goal** at (50, 50) for all trials.
  Obstacles are re-generated randomly each trial, so the A* path and force field
  are unique to each trial despite the fixed endpoints.
- The **A* + SFM robot** (agent 1) and **SFM-only robot** (agent 2) run simultaneously
  in the same environment and among the same 48 pedestrians, ensuring a controlled
  comparison under identical conditions.
- **Pedestrians** (agents 3–50) are initialized with random positions and velocities
  drawn from a standard normal distribution, and are assigned random goal positions.
  They are governed by the same SFM force equations as the robots.
- Each trial runs for a maximum of **3000 time steps** (30 simulated seconds).

---

## Reproducibility

Results vary between runs because each trial draws a fresh random seed. Since every random
element — obstacle polygons, pedestrian positions, velocities, and goals — is drawn from
MATLAB's global generator, the **integer seed fully determines the trial**: `rng(seed)`
regenerates the exact same environment. So each run only needs to save its seeds, not the
generated obstacles or pedestrians.

Each experiment writes two things:

- A bulky `.mat` of full results (`success_rate_results.mat`, `path_distance_results.mat`) —
  **not** tracked in git.
- A small per-trial seed log under **`runs/`** — **committed** to git as the reproducibility
  record (`runs/success_rate_seeds.csv`, `runs/path_distance_seeds.csv`). See `runs/README.md`
  for the column format.

To reproduce trial `i` exactly, read its seed from the log (or the `.mat`) and re-run:

```matlab
load('success_rate_results.mat', 'trial_seeds');
rng(trial_seeds(i));   % then call Success()
```

The aggregate statistics are averages over many trials and are stable across seeds.

---

## License

MIT License. See LICENSE file for details.
