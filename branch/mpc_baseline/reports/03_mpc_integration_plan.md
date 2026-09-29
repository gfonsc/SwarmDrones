# MPC Integration Plan

## 1. Which TCC files will be copied into `copied_from_tcc`

Copied files:

- `crazyflie_params.m`
- `linearize_quadrotor.m`
- `quadrotor_dynamics.m`
- `design_mpc_quadrotor.m`
- `mpc_controller_mosek.m`
- `mpc_controller_mosek_v2.m`
- `test_mpc_quadrotor.m`
- `test_mpc_mosek_v2.m`
- `mpc_top10_evaluation.m`

These are kept as auditable branch-side copies. The main branch controller is implemented in new wrapper files rather than by altering the originals in place.

## 2. Which files will be reused without modification

Read-only reuse from `SwarmDrones/branch`:

- existing DeePC result files under `branch/results/`
- `branch_compute_safety_margin.m`
- `branch_linearize_obstacle_constraints.m`
- the original labyrinth scenario and geometry inside the `SwarmDrones` project

Read-only reuse from `TCC/mpc-matlab` copied into the branch:

- `mpc_controller_mosek_v2.m` as the structural reference for the condensed QP

## 3. Which files require adaptation

Adaptation is required for:

- the state-space model used by the optimizer
- the reference interface
- obstacle constraints
- result packaging
- plotting and metric generation

## 4. What the MPC state vector will be in the branch

For fair comparison with the existing labyrinth DeePC branch, the branch MPC uses the same plant state as the original labyrinth simulation:

`x = [x, y, z, vx, vy, vz, yaw, yaw_rate]'`

This is not the 12-state TCC quadrotor model. It is the exact branch plant state used by `simulateLabyrinthDroneStep`.

## 5. What the MPC input vector will be in the branch

The branch MPC acts on the same labyrinth command interface as the DeePC branch:

`u = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]'`

## 6. How the MPC output will be compared to DeePC output

The comparison uses the same branch quantities for both controllers:

- reference position trajectory
- realized position trajectory
- command histories
- safety margin
- solver time
- feasibility

## 7. Whether the MPC will produce motor speeds directly

No.

The adapted branch MPC does not produce physical motor speeds directly because the labyrinth plant itself is command-based. A separate `mpc_motor_mapping` function is included only for traceable conversion when thrust/torque inputs are present in future extensions.

## 8. How control effort will be compared

In the present branch comparison:

- DeePC and MPC control effort are compared on the four branch command channels;
- plots keep four channels for consistency;
- reports explicitly state that these are command efforts, not measured rotor speeds.

## 9. Whether obstacle constraints can be included directly

Yes, in the adapted branch controller.

Because the branch MPC uses a linear prediction model, obstacle constraints can be added as affine inequalities over predicted positions after linearization around a nominal predicted trajectory.

## 10. Which obstacle formulation will be used

Implementation order:

1. baseline MPC tracking without obstacle-aware correction;
2. obstacle-aware MPC with linearized inequalities and nonnegative slack;
3. branch-side iterative successive linearization.

The implemented obstacle mode is:

- linearized constraints;
- soft slack variables;
- quadratic slack penalty.

## 11. What the first working implementation should be

The first working target is:

- reproduce the same labyrinth scenario and path-following setup used by the existing DeePC branch;
- run a feasible MPC baseline on the same kinematic plant and initial condition;
- save comparable metrics and figures.

After that baseline works, the obstacle-aware layer is added using the same geometry and safety-margin definition already used by the DeePC branch.
