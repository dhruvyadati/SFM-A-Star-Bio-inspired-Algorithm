# Parameter screen results

Exploratory development-set screen; 100 shared scenarios per setting and arm.
Counts are out of 100. Effects are descriptive, without confirmatory inference.

| Setting | Arm | Safe | Unsafe arrival | Deadlock | Timeout | Any contact |
|---|---|---:|---:|---:|---:|---:|
| nominal | hybrid | 27 | 71 | 2 | 0 | 73 |
| nominal | hybrid-ttc | 33 | 50 | 17 | 0 | 61 |
| ped_speed_low | hybrid | 31 | 66 | 3 | 0 | 68 |
| ped_speed_low | hybrid-ttc | 42 | 34 | 24 | 0 | 46 |
| ped_speed_high | hybrid | 37 | 62 | 1 | 0 | 63 |
| ped_speed_high | hybrid-ttc | 38 | 45 | 17 | 0 | 57 |
| obs_radius_low | hybrid | 28 | 69 | 3 | 0 | 72 |
| obs_radius_low | hybrid-ttc | 33 | 50 | 17 | 0 | 61 |
| obs_radius_high | hybrid | 27 | 71 | 2 | 0 | 73 |
| obs_radius_high | hybrid-ttc | 33 | 50 | 17 | 0 | 61 |
| speed_cap_low | hybrid | 28 | 69 | 3 | 0 | 72 |
| speed_cap_low | hybrid-ttc | 33 | 51 | 16 | 0 | 60 |
| speed_cap_high | hybrid | 27 | 70 | 3 | 0 | 72 |
| speed_cap_high | hybrid-ttc | 41 | 43 | 16 | 0 | 53 |

| Setting | TTC safe-completion difference (pp) | Change from nominal (pp) |
|---|---:|---:|
| nominal | +6 | +0 |
| ped_speed_low | +11 | +5 |
| ped_speed_high | +1 | -5 |
| obs_radius_low | +5 | -1 |
| obs_radius_high | +6 | +0 |
| speed_cap_low | +5 | -1 |
| speed_cap_high | +14 | +8 |

## Condition summaries

Each condition has 25 scenarios per setting and arm. These are descriptive subsets.

| Setting | Condition | SFM safe | SFM + TTC safe | Paired difference (pp) |
|---|---|---:|---:|---:|
| nominal | baseline | 8 | 5 | -12 |
| nominal | bigObs | 8 | 7 | -4 |
| nominal | cluttered | 9 | 17 | +32 |
| nominal | dense | 2 | 4 | +8 |
| ped_speed_low | baseline | 9 | 12 | +12 |
| ped_speed_low | bigObs | 5 | 7 | +8 |
| ped_speed_low | cluttered | 14 | 16 | +8 |
| ped_speed_low | dense | 3 | 7 | +16 |
| ped_speed_high | baseline | 12 | 13 | +4 |
| ped_speed_high | bigObs | 10 | 10 | +0 |
| ped_speed_high | cluttered | 8 | 10 | +8 |
| ped_speed_high | dense | 7 | 5 | -8 |
| obs_radius_low | baseline | 8 | 4 | -16 |
| obs_radius_low | bigObs | 8 | 8 | +0 |
| obs_radius_low | cluttered | 9 | 17 | +32 |
| obs_radius_low | dense | 3 | 4 | +4 |
| obs_radius_high | baseline | 8 | 5 | -12 |
| obs_radius_high | bigObs | 8 | 7 | -4 |
| obs_radius_high | cluttered | 9 | 17 | +32 |
| obs_radius_high | dense | 2 | 4 | +8 |
| speed_cap_low | baseline | 7 | 5 | -8 |
| speed_cap_low | bigObs | 9 | 6 | -12 |
| speed_cap_low | cluttered | 10 | 16 | +24 |
| speed_cap_low | dense | 2 | 6 | +16 |
| speed_cap_high | baseline | 7 | 7 | +0 |
| speed_cap_high | bigObs | 7 | 8 | +4 |
| speed_cap_high | cluttered | 9 | 18 | +36 |
| speed_cap_high | dense | 4 | 8 | +16 |
