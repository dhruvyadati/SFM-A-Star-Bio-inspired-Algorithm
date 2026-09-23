# Navigation parameter sensitivity

Frozen before execution on 2026-09-13. This is a supplementary, exploratory
one-at-a-time analysis planned after inspecting earlier outcomes. It is not an
independent confirmatory test and will not replace the existing primary analysis.

Use the first 25 scenarios per condition in lexicographic filename order from
the existing 300 development exports: 100 scenarios, shared by every setting.
Run guided SFM and guided SFM with TTC for seven settings (1,400 trials):
nominal; pedestrian desired speed 1.0 and 1.7 m/s (nominal 1.34);
obstacle influence radius 2.0 and 4.0 m (nominal 3.0);
shared agent speed cap 1.5 and 2.5 m/s (nominal 2.0).
These are computational perturbations, not empirically calibrated human speeds.
Change only the named parameter. Keep initial states, timestep, physical duration,
goals, waypoint route, TTC parameters and every other setting unchanged. The speed
cap applies to both robot and pedestrians. Nominal trials are newly run using the
same source snapshot as perturbed trials. No tuning or outcome-based selection.

Record the usual sampled-contact endpoint: safe completion requires stopped goal
arrival and zero recorded contacts across the trial. Also retain unsafe arrivals,
deadlock, timeout, contact count, minimum surface gaps, steps and path length.
Report counts and percentages for both arms, paired TTC differences, and their
change relative to nominal. This descriptive screen does not establish robustness
or parameter interactions. Report errors rather than silently dropping scenarios;
no automatic failed-trial retries. Report incomplete execution if interrupted.

Hash all simulation sources, scenario files, protocol and runner before execution.
Save each result separately and atomically, allowing resumption without rerunning
completed trials. Verify source and input hashes before resuming.
