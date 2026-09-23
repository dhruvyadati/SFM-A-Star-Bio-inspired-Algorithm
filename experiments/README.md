# Additional navigation experiments

Saved inputs and results extend the core outputs under `v2/runs/v2/`:

| Directory | Contents |
|---|---|
| `orca_robot` | 3,000 development trials, fixed margin selection, 1,000 evaluation trials and paired comparisons |
| `parameter_sensitivity` | 1,400 trials across seven settings and two guided controllers |
| `clearance_diagnostics` | Six-controller sampled-clearance analysis and eight saved trajectory replays |
| `figure_layout` | Rendering scripts and saved figures |

## Verify and analyze saved results

From the repository root, install `experiments/requirements.txt` in an isolated
Python environment, then run:

```sh
python experiments/verify_files.py
python experiments/orca_robot/analyze_results.py
python experiments/parameter_sensitivity/analyze.py
python experiments/clearance_diagnostics/analyze.py
```

The analysis commands validate recorded inputs and regenerate derived summaries;
they do not run new simulation trials. Verify checksums before analysis. The
recorded analysis environment uses Python 3.13 and NumPy 2.3.1. Plotting uses
Matplotlib 3.10.8. Other versions can produce different formatting or rounding.

To regenerate plots from saved data:

```sh
python experiments/clearance_diagnostics/plot.py
python experiments/figure_layout/plot.py
```

Simulation replay additionally requires the recorded runtime and dependencies,
including Python-RVO2 for ORCA. See the per-experiment protocols and manifests.
Use a separate working copy for replay and preserve the archived outputs.

## Relocation and provenance

These directories were relocated beside the core `py/` and `v2/` directories.
All raw trial and trajectory files retain their original bytes. Script changes
only resolve the repository-local paths. The active parameter-screen and trace
manifests identify the relocated runner hashes. Their original manifests and
runner source files are retained under `provenance/` as read-only historical
records, not executable entry points. `provenance/relocation.json` records the
old and new hashes; historical timestamps describe the original execution.

Recomputed ORCA summaries identify the relocated analysis source hash. Clearance
summaries use repository-relative input paths. Neither changes numerical results.
`FILES_SHA256.json` inventories the distributable experiment files.

Original software uses the repository MIT license; original data and
 documentation use `DATA_LICENSE.md`. Existing third-party terms remain applicable.
