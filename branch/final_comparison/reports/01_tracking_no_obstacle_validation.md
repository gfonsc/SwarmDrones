# Scenario A - No-Obstacle Tracking Validation

- DeePC run successful: `1`
- MPC run successful: `1`
- Same reference used: `yes`
- Same initial condition used: `yes`
- Same number of samples: `420`
- Sampling time: `0.100 s`
- Time vectors aligned: `1`

- Reference source: `Fixed resampling of original labyrinth mission setpoints`
- Labyrinth walls preserved for plotting/context: `yes`
- Cylindrical obstacles active: `no`

## Metrics

| Controller | RMSE_total (m) | Max error (m) | Final error (m) | Mean solve time (s) | Feasibility |
|---|---:|---:|---:|---:|---:|
| DeePC | 0.045850 | 0.162657 | 0.049711 | 0.054911 | 1.000 |
| MPC | 0.063049 | 0.101611 | 0.015574 | 0.001098 | 1.000 |

## Conclusion

Both controllers ran over the same nominal labyrinth setpoint reference and time window. Based on RMSE_total, `DeePC` tracks better in Scenario A.

Motor-effort comparison type: `equivalent normalized command effort`. Valid for article as normalized equivalent effort: `1`.
