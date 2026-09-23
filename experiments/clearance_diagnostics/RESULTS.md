# Sampled clearance diagnostics

Post hoc descriptive analysis of minimum recorded robot surface gaps.
Each arm uses the same 500 scenarios. No uncertainty or confirmatory test is added.
A contact-free close pass means no recorded contact and at least one sampled
surface gap at or below 0.15 m (an operational threshold, not validated comfort).
A margin encounter can include contact. Trials have different durations.
These summaries do not detect collisions between samples.

| Arm | Any contact | Contact-free close pass | Any gap <= 0.15 m | Any gap <= 0.40 m | Median minimum pedestrian gap (m) | Median minimum obstacle gap (m) |
|---|---:|---:|---:|---:|---:|---:|
| sfm | 302 | 191 | 493 | 500 | -0.013 | 0.124 |
| sfm-ttc | 205 | 196 | 401 | 500 | 0.107 | 0.149 |
| hybrid | 321 | 73 | 394 | 490 | -0.048 | 0.319 |
| hybrid-ttc | 263 | 64 | 327 | 490 | -0.011 | 0.332 |
| orca-direct | 283 | 217 | 500 | 500 | 0.020 | -3.75e-07 |
| orca-guided | 60 | 431 | 491 | 498 | 0.021 | 3.61e-05 |
