# Article-Ready Results Summary

Scenario A evaluated trajectory tracking through the nominal labyrinth setpoints, with labyrinth walls drawn and no cylindrical obstacles active. DeePC obtained `RMSE_total = 0.0459 m`, while MPC obtained `RMSE_total = 0.0630 m` over the same time interval.

Scenario B used the original labyrinth training reference for both controllers, with the branch cylindrical obstacles active. DeePC obtained `RMSE_total = 0.0568 m` and `min_h = -0.0185 m`; MPC obtained `RMSE_total = 0.0679 m` and `min_h = 0.0101 m`.

Tracking should be interpreted using `RMSE_total`, maximum error, and final error from `final_metrics_full.csv`, because both controllers were evaluated on executed trajectories rather than predictions.

Safety should be described carefully. A positive `min_h` would indicate clearance from the active cylindrical obstacles; a negative value means residual safety-margin violation remains and should not be described as successful obstacle avoidance.

Control effort is reported as equivalent normalized command effort. This supports a relative smoothness and effort comparison inside the branch simulation, but it is not a physical rotor-speed measurement.

The main limitation is that the final comparison uses the SwarmDrones branch kinematic command interface, not a full physical motor-speed quadrotor model.
