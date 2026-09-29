# MPC Source Audit

## 1. What files exist in `TCC/mpc-matlab`?

Relevant files found during the audit:

- `crazyflie_params.m`
- `linearize_quadrotor.m`
- `quadrotor_dynamics.m`
- `design_mpc_quadrotor.m`
- `mpc_controller_mosek.m`
- `mpc_controller_mosek_v2.m`
- `test_mpc_quadrotor.m`
- `test_mpc_mosek_v2.m`
- `mpc_parameter_sweep.m`
- `mpc_top10_evaluation.m`
- several `.mat` result files and `.png` summary figures

## 2. Which files are main scripts?

Main scripts:

- `test_mpc_quadrotor.m`
- `test_mpc_mosek_v2.m`
- `mpc_parameter_sweep.m`
- `mpc_top10_evaluation.m`
- `run_best_mpc_animation.m`

## 3. Which files are reusable functions?

Reusable functions:

- `crazyflie_params.m`
- `linearize_quadrotor.m`
- `quadrotor_dynamics.m`
- `design_mpc_quadrotor.m`
- `mpc_controller_mosek.m`
- `mpc_controller_mosek_v2.m`

## 4. What model is used by the MPC?

The source project uses a model-based quadrotor controller with two layers:

- a continuous-time linearized hover model in `linearize_quadrotor.m`;
- a nonlinear simulation model in `quadrotor_dynamics.m`.

The practical MPC controllers discretize the linearized model and solve a finite-horizon QP.

Model characterization:

- model class: linearized around hover
- state-space: yes
- base form: continuous-time, then discretized
- plant dimension: 12 states, 4 inputs

## 5. What state vector is used?

The linearized and nonlinear source files use:

`x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]'`

## 6. What input vector is used?

The source MPC acts on a 4-channel control vector derived from total thrust and torques:

`u = [DeltaT, tau_phi, tau_theta, tau_psi]'`

or, in the nonlinear dynamics file,

`u = [T, tau_phi, tau_theta, tau_psi]'`

This is not the same as the labyrinth branch command interface `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`.

## 7. What outputs are tracked?

The source project tracks position directly and also penalizes velocity and angle deviations.

- `design_mpc_quadrotor.m` uses `C` for `[px, py, pz, phi, theta, psi]`
- `mpc_controller_mosek_v2.m` builds a full-state cost with:
  - position tracking
  - velocity tracking
  - angle regularization
  - input penalty
  - input-rate penalty
  - terminal cost

## 8. What constraints are implemented?

Implemented in the source project:

- box bounds on the 4 control inputs
- angle-related bounds in the MATLAB MPC Toolbox design
- quadratic stage cost
- terminal cost
- rate penalty

Not found in the source project:

- obstacle constraints
- wall constraints
- safety-margin constraints
- barrier functions
- signed-distance constraints

## 9. Which solver is used?

Two solver paths exist:

- custom condensed QP with MOSEK in `mpc_controller_mosek*.m`
- MATLAB MPC Toolbox in `design_mpc_quadrotor.m` and `test_mpc_quadrotor.m`

This environment also exposes `quadprog`, `mpc`, and `mosekopt`, so the branch can use either MOSEK or `quadprog`.

## 10. Whether obstacle constraints already exist

No reusable obstacle-aware MPC constraints were found in `TCC/mpc-matlab`.

The source project must therefore be extended on the branch side if we want obstacle-aware MPC in the labyrinth comparison.

## 11. Whether the implementation can be adapted to the SwarmDrones branch

Yes, but not by reusing the source dynamics unchanged.

The source MPC assumes a 12-state quadrotor model with thrust/torque inputs, whereas the current labyrinth DeePC branch uses:

- an 8-state kinematic surrogate plant;
- inputs `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`;
- output-driven path following in a bounded labyrinth.

The safest branch strategy is:

1. reuse the source MPC condensed-QP structure, weight layout, and solver style;
2. adapt the prediction model to the labyrinth plant dynamics;
3. keep the same labyrinth scenario, reference path, safety geometry, and initial condition as the DeePC branch.

## 12. What adapters are necessary?

Required branch-side adapters:

- a path/setup layer that loads the existing branch labyrinth scenario;
- an adapted discrete-time MPC model for the labyrinth surrogate plant;
- a common metric function aligned with article-style evaluation;
- a safety-margin wrapper that reuses the DeePC branch safety geometry;
- an obstacle-aware MPC wrapper with linearized wall/cylinder constraints and slack;
- plotting utilities that use the same scenario and comparison style as the branch DeePC results.

## 13. Reuse recommendation

Reuse read-only from `TCC/mpc-matlab`:

- `crazyflie_params.m` for documentation of the original source assumptions;
- `linearize_quadrotor.m` and `quadrotor_dynamics.m` for the source audit and future higher-fidelity extension;
- `mpc_controller_mosek_v2.m` as the main structural template for the branch condensed-QP tracking controller.

Adapt on the branch only:

- the model matrices;
- the reference interface;
- the obstacle-aware constraints;
- the metric and plotting layer.
