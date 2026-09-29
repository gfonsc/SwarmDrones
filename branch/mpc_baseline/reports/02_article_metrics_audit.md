# Article Metrics Audit

## 1. Metrics explicitly visible in `2026_CBA_GFonseca.md`

The current article draft explicitly uses or reports:

- `RMSE_x`
- `RMSE_y`
- `RMSE_z`
- `RMSE_total`
- best open-loop prediction RMSE
- mean RMSE over windows
- Hankel condition number

The article also discusses:

- trajectory generalization
- obstacle avoidance
- solver tractability of the augmented QP

## 2. Metrics not fully formalized in the draft but needed for controller comparison

The draft does not yet define a full closed-loop DeePC-vs-MPC comparison table. For the labyrinth branch comparison, the following extra metrics are recommended so the evaluation remains consistent with the TCC and the new obstacle-aware setting:

- `MAE_x`, `MAE_y`, `MAE_z`
- maximum tracking error
- final tracking error
- control energy
- control variation
- minimum obstacle distance
- minimum safety margin `h(k)`
- number of safety violations
- maximum violation depth
- mean solver time
- maximum solver time
- feasibility rate

## 3. Why these extra metrics are defensible

These additions do not change the article narrative. They extend the existing evaluation style from prediction-only DeePC to closed-loop constrained tracking:

- RMSE keeps continuity with the current draft tables.
- safety-margin metrics are necessary once obstacle constraints are introduced.
- control-energy and control-variation metrics provide a fairer DeePC-vs-MPC discussion.
- solver-time and feasibility metrics support the article's practical tractability claims.

## 4. Recommended comparison metric set

The branch baseline uses the following metric set whenever data are available:

1. `RMSE_x`
2. `RMSE_y`
3. `RMSE_z`
4. `RMSE_total`
5. `MAE_x`
6. `MAE_y`
7. `MAE_z`
8. `max_error_x`
9. `max_error_y`
10. `max_error_z`
11. `max_error_total`
12. `final_error_total`
13. `control_energy_total`
14. `control_energy_per_motor`
15. `control_variation_total`
16. `control_variation_per_motor`
17. `min_distance_to_obstacle`
18. `min_h`
19. `num_safety_violations`
20. `max_violation_depth`
21. `mean_solver_time`
22. `max_solver_time`
23. `feasibility_rate`

## 5. Important interpretation caveat for the labyrinth branch

The current labyrinth branch does not command physical motor angular velocities. Both the DeePC branch and the adapted branch-side MPC operate on the original SwarmDrones labyrinth command interface:

`[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`

Therefore:

- control-energy and control-variation metrics in this branch refer to the four command channels, not real motor power;
- if a future article section compares against the IDSIA motor-speed DeePC results, that distinction must be stated clearly.
