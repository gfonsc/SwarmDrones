# MPC Results Summary

## 1. Did the MPC baseline run?

Yes.

The branch-side baseline MPC ran successfully on the same labyrinth scenario, reference path, and initial condition used by the existing DeePC branch. The result file is:

- `results/mpc_baseline_results.mat`

Baseline metrics:

- `RMSE_total = 0.22797 m`
- `min_h = -0.04562 m`
- `num_safety_violations = 3`
- `mean_solver_time = 0.00184 s`
- `feasibility_rate = 1.0`

## 2. Did the MPC obstacle-aware version run?

Yes.

The branch-side obstacle-aware MPC ran in two forms:

- single-pass soft linearized constraints
- iterative successive linearization

The result file is:

- `results/mpc_obstacle_results.mat`

Soft obstacle-aware metrics:

- `RMSE_total = 0.22714 m`
- `min_h = -0.01624 m`
- `num_safety_violations = 1`
- `mean_solver_time = 0.00254 s`
- `feasibility_rate = 1.0`

Iterative obstacle-aware metrics:

- `RMSE_total = 0.22715 m`
- `min_h = -0.01619 m`
- `num_safety_violations = 1`
- `mean_solver_time = 0.00205 s`
- `feasibility_rate = 1.0`

## 3. What model and solver were used?

The branch-side MPC reuses the condensed-QP structure of the TCC source `mpc_controller_mosek_v2.m`, but the prediction model was adapted to the SwarmDrones labyrinth plant so the comparison stayed on the same closed-loop simulation used by the DeePC branch.

Model used in the branch:

- discrete-time 8-state kinematic labyrinth plant
- state:
  - `[x, y, z, vx, vy, vz, yaw, yaw_rate]`
- input:
  - `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`

Solver path:

- `solveQP(..., solver='auto')`
- this environment selected `mosek` for the branch MPC solves

## 4. What input representation was used?

The branch MPC uses the same command interface as the SwarmDrones labyrinth branch:

- `vx_cmd`
- `vy_cmd`
- `vz_cmd`
- `yaw_rate_cmd`

This is not a physical motor-speed controller.

## 5. How were MPC inputs converted to four motor efforts?

For the present labyrinth comparison, they were not converted to real motor speeds because the underlying branch plant is command-based, not rotor-based.

The branch still includes `functions/mpc_motor_mapping.m` for future thrust/torque-to-motor conversion, but the current paper figures use the four native command channels directly and label them as such.

## 6. What scenarios were tested?

The branch comparison covers:

- `Scenario 0 baseline`: branch labyrinth tracking without obstacle-aware correction
- `Scenario 1 obstacle`: soft linearized obstacle-aware MPC
- `Scenario 2 obstacle`: iterative obstacle-aware MPC

These use the same labyrinth geometry already present in the existing DeePC branch rather than a new external scenario catalog.

## 7. What metrics were obtained?

The full tables are:

- `tables/mpc_metrics.csv`
- `tables/deepc_mpc_comparison_metrics.csv`
- `tables/paper_comparison_table.csv`

Key comparison numbers:

- DeePC baseline: `RMSE_total = 0.26787 m`, `min_h = -0.10107 m`, `11` safety violations
- MPC baseline: `RMSE_total = 0.22797 m`, `min_h = -0.04562 m`, `3` safety violations
- DeePC iterative obstacle: `RMSE_total = 0.26810 m`, `min_h = -0.09849 m`, `11` safety violations
- MPC iterative obstacle: `RMSE_total = 0.22715 m`, `min_h = -0.01619 m`, `1` safety violation

## 8. What figures were generated?

Scenario figures:

- `figures/scenario_00/`
- `figures/scenario_01/`
- `figures/scenario_02/`

Comparison figures:

- `figures/comparison/scenario_00/`
- `figures/comparison/scenario_01/`
- `figures/comparison/scenario_02/`

Paper-ready figures:

- `figures/paper_ready/paper_2d_trajectory_deepc_vs_mpc_obstacles.png`
- `figures/paper_ready/paper_3d_trajectory_deepc_vs_mpc_obstacles.png`
- `figures/paper_ready/paper_control_efforts_4_motors_deepc_vs_mpc.png`
- `figures/paper_ready/paper_xyz_deepc_vs_mpc.png`
- `figures/paper_ready/paper_euler_angles_deepc_vs_mpc.png`
- `figures/paper_ready/paper_safety_margin_deepc_vs_mpc.png`
- `figures/paper_ready/paper_tracking_error_deepc_vs_mpc.png`

## 9. What failed or required approximation?

No run failed structurally, but some important approximations remain:

- the branch MPC is adapted to the 8-state labyrinth surrogate, not the full 12-state TCC quadrotor model;
- obstacle constraints are linearized and softened with slack;
- obstacle-aware MPC currently enforces cylindrical obstacle constraints but does not yet eliminate all negative safety-margin samples;
- yaw is available, but roll and pitch are unavailable in the branch plant and are therefore shown as unavailable rather than fabricated.

## 10. Is the MPC comparison fair?

It is fair at the branch-scenario level because both controllers use:

- the same labyrinth geometry;
- the same reference path logic;
- the same initial condition;
- the same plant surrogate;
- the same safety-margin evaluation.

It is not a full physics-level comparison between motor-speed DeePC and a 12-state quadrotor MPC, so that distinction must be stated in the article.

## 11. What limitations must be mentioned in the article?

The article should explicitly mention that:

- the present MPC baseline is an adapted branch-side model-based controller for the SwarmDrones labyrinth surrogate;
- both DeePC and MPC are compared here on the same command-based plant, not on the IDSIA motor-speed plant;
- the obstacle-aware variants still retain slightly negative `h(k)` in the most constrained segment;
- the current branch therefore supports a transparent comparative baseline, but not a claim of guaranteed collision-free traversal.
