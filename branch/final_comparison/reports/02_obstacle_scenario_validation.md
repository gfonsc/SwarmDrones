# Scenario B - Obstacle Tracking Validation

- Obstacles active: `yes`
- Number of obstacles: `4`
- Same obstacles used by DeePC and MPC: `yes`
- Same reference used: `yes`
- Reference source: `Fixed resampling of original labyrinth training setpoints with obstacle/labyrinth deviations`
- Same initial condition used: `yes`
- Time vectors aligned: `1`
- Labyrinth walls preserved for plotting/context: `yes`
- Safety margin uses cylindrical obstacles only: `yes`
- Wall constraints included in final safety metric: `no`

## Metrics

| Controller | RMSE_total (m) | min h (m) | Violations | Mean solve time (s) | Feasibility |
|---|---:|---:|---:|---:|---:|
| DeePC | 0.056762 | -0.018496 | 3 | 0.058437 | 1.000 |
| MPC | 0.067856 | 0.010107 | 0 | 0.001384 | 1.000 |

## Conclusion

- DeePC satisfies `h(k) >= 0`: `no`
- MPC satisfies `h(k) >= 0`: `yes`
- Better safety margin in Scenario B: `MPC`
- Obstacle avoidance is inside both optimization problems as soft linearized constraints with slack.
- Motor-effort comparison type: `equivalent normalized command effort`; valid as normalized equivalent effort: `1`.
