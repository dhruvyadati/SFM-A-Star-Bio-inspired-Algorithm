# ORCA robot results

Descriptive post-evaluation paired percentile intervals, 10000 stratified resamples; no multiplicity-adjusted or confirmatory inference

| Controller | Safe | Unsafe arrival | Deadlock | Timeout | Any contact |
|---|---:|---:|---:|---:|---:|
| sfm | 21/500 (4.2%) | 96/500 (19.2%) | 365/500 (73.0%) | 18/500 (3.6%) | 302/500 (60.4%) |
| sfm-ttc | 19/500 (3.8%) | 33/500 (6.6%) | 440/500 (88.0%) | 8/500 (1.6%) | 205/500 (41.0%) |
| hybrid | 176/500 (35.2%) | 314/500 (62.8%) | 10/500 (2.0%) | 0/500 (0.0%) | 321/500 (64.2%) |
| hybrid-ttc | 215/500 (43.0%) | 215/500 (43.0%) | 70/500 (14.0%) | 0/500 (0.0%) | 263/500 (52.6%) |
| orca-direct | 38/500 (7.6%) | 73/500 (14.6%) | 370/500 (74.0%) | 19/500 (3.8%) | 283/500 (56.6%) |
| orca-guided | 426/500 (85.2%) | 51/500 (10.2%) | 23/500 (4.6%) | 0/500 (0.0%) | 60/500 (12.0%) |

## orca-guided minus hybrid
- safe: +50.0 percentage points [+45.0, +55.0]
- unsafe: -52.6 percentage points [-57.4, -47.8]
- deadlock: +2.6 percentage points [+0.6, +4.6]
- contact: -52.2 percentage points [-57.2, -47.2]
- Safe gains/losses: 274/24
- Safe delta excluding three inspected scenarios: +49.70 percentage points (497 pairs)

## orca-guided minus hybrid-ttc
- safe: +42.2 percentage points [+36.8, +47.4]
- unsafe: -32.8 percentage points [-37.8, -27.8]
- deadlock: -9.4 percentage points [-13.0, -6.0]
- contact: -40.6 percentage points [-45.6, -35.6]
- Safe gains/losses: 241/30
- Safe delta excluding three inspected scenarios: +41.85 percentage points (497 pairs)

## orca-direct minus sfm
- safe: +3.4 percentage points [+0.8, +6.0]
- unsafe: -4.6 percentage points [-8.6, -0.8]
- deadlock: +1.0 percentage points [-3.0, +5.0]
- contact: -3.8 percentage points [-9.4, +2.0]
- Safe gains/losses: 30/13
- Safe delta excluding three inspected scenarios: +3.42 percentage points (497 pairs)

## orca-direct minus sfm-ttc
- safe: +3.8 percentage points [+1.4, +6.2]
- unsafe: +8.0 percentage points [+4.6, +11.4]
- deadlock: -14.0 percentage points [-17.8, -10.2]
- contact: +15.6 percentage points [+9.8, +21.4]
- Safe gains/losses: 30/11
- Safe delta excluding three inspected scenarios: +3.82 percentage points (497 pairs)
