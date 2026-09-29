# DeePC vs MPC Fair Comparison Report

## Scenarios

- Scenario A: nominal labyrinth setpoint tracking with no cylindrical obstacles active.
- Scenario B: original labyrinth training reference with the branch cylindrical obstacles active.

Within each scenario, DeePC and MPC use the same reference, same initial condition, same sampling time, and same fixed time window.
The labyrinth walls are preserved and drawn in both final trajectory figures.
Motor efforts are equivalent normalized command efforts, not physical motor speeds.

## Main Results

| Scenario | Controller | RMSE_total | min h | Control energy | Mean solve time |
|---|---|---:|---:|---:|---:|
| A | DeePC | 0.045850 | NaN | 4.483622 | 0.054911 |
| A | MPC | 0.063049 | NaN | 6.648569 | 0.001098 |
| B | DeePC | 0.056762 | -0.018496 | 5.547080 | 0.058437 |
| B | MPC | 0.067856 | 0.010107 | 7.071305 | 0.001384 |

- Better Scenario A tracking by RMSE: `DeePC`
- Better Scenario B tracking by RMSE: `DeePC`
- Better Scenario B safety margin: `MPC`
- Lower Scenario B equivalent control energy: `DeePC`
- Computationally heavier in Scenario B: `DeePC`

## Article Claims

Safe to write: within each scenario, DeePC and MPC use identical references, time bases, initial conditions, and obstacle definitions.
Safe to write: the labyrinth geometry is drawn in both Scenario A and Scenario B figures.
Safe to write: motor effort is reported as equivalent normalized command effort, not physical rotor speed.
Do not claim strict obstacle avoidance if either Scenario B `min_h` is negative.
Do not claim physical motor-speed comparison for this branch implementation.
