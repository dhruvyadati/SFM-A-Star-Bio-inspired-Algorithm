# Execution record

The initial execution saved 1,003 of 1,400 trials before stopping. The final
status label from that process was stale. On 2026-09-16, the run lock was checked
and found free, and execution was resumed using the existing runner and manifest.
All source, input, protocol, settings and runtime hashes were checked by the
runner before resumption. Completed trial files were retained and skipped; the
remaining 397 jobs were scheduled. No completed trial was retried, no setting was
changed, and no results were used to change the plan.

The elapsed time in status.json refers to the most recent execution session,
not total elapsed time across both sessions. run.log contains progress from both.
The status file and per-trial records determine whether execution is complete;
a stored running label alone is not proof that a process is still active.
