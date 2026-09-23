# Sampled clearance and trajectory diagnostics

`analyze.py` describes the six matched robot-controller arms using saved minimum
surface gaps. Its `analysis.json` records hashes for all source result files.
Close passes are counted only in trials with zero recorded contacts, using a
minimum surface gap at or below 0.15 m. Margin encounter counts include contact
trials. All counts use the full 500 trials per arm. These are post hoc operational
thresholds, not validated comfort metrics or continuous-collision checks.

`trace.py` replays guided SFM with and without TTC on the first lexicographic
confirmation scenario per condition. The selection is illustrative, not random
or representative. An observer wraps the existing force computation, records its
inputs and gap outputs, and returns the original outputs unchanged. No original
simulation source was edited. All eight replays must match the saved outcomes,
contact counts, step counts, path lengths and minimum gaps before they are saved.

Position snapshots use the pre-update state at 0.1 s intervals. Gap arrays retain
every pre-update 0.01 s sample. The final post-update state is outside this sampled
contact definition. `plot.py` draws all pedestrian tracks, waypoints, the robot
track, separate magnified panels of agent footprints at the closest logged pedestrian approach, a robot
radius plus 0.15 m ring, and sampled gap histories. Gap axes use a symmetric
logarithmic scale with a linear region within +/-0.2 m.

Run `python analyze.py` and `python plot.py` from any directory with NumPy and
Matplotlib installed to reproduce these outputs. `trace.py` requires the recorded
simulation environment; saved traces avoid the need to rerun the simulations.

Figures use panels (a)-(f): trajectories, separate magnified encounters, and gap
histories for the two controllers. Legends lie outside the data axes.
