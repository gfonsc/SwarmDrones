# Constraint Implementation Audit

## Scope

This audit checks whether the professor-proposed obstacle-avoidance constraints are actually implemented inside the current MATLAB optimization problems in the branch folder.

Controllers audited:

- DeePC branch files:
  - `branch/functions/branch_solve_obstacle_deepc.m`
  - `branch/functions/branch_build_obstacle_constraints.m`
  - `branch/functions/branch_linearize_obstacle_constraints.m`
  - `branch/scripts/run_03_branch_soft_obstacle_deepc.m`
  - `branch/scripts/run_04_branch_iterative_obstacle_deepc.m`
- MPC branch files:
  - `branch/mpc_baseline/functions/mpc_solve_tracking.m`
  - `branch/mpc_baseline/functions/mpc_solve_obstacle_aware.m`
  - `branch/mpc_baseline/functions/mpc_run_labyrinth_rollout.m`
  - `branch/mpc_baseline/scripts/run_02_mpc_labyrinth_baseline.m`
  - `branch/mpc_baseline/scripts/run_03_mpc_labyrinth_obstacle_aware.m`
  - copied TCC source templates inside `branch/mpc_baseline/copied_from_tcc/`

Saved branch results used in this audit:

- DeePC soft: `min h = -0.098363 m`
- DeePC iterative: `min h = -0.098489 m`
- MPC baseline: `min h = -0.045625 m`
- MPC soft: `min h = -0.016242 m`
- MPC iterative: `min h = -0.016185 m`

In all obstacle-aware saved runs, the worst violation comes from obstacle `4`.

---

## DeePC Audit

### A. Is obstacle avoidance included inside the optimization problem?

Yes.

`branch_solve_obstacle_deepc.m` augments the DeePC QP with obstacle inequalities:

- original decision variable: `g`
- augmented decision variable: `[g; s_obs]`

The file constructs:

- `H_aug = blkdiag(H_g, 2 * rhoObs * I)`
- `Aineq_aug = [A_model, 0; A_obs, -I; 0, -I]`

So obstacle constraints are not just checked afterward; they are added directly to the QP.

### B. Is it hard, soft, linearized, or only a cost penalty?

It is implemented as:

- soft constraint
- linearized constraint
- with a quadratic slack penalty

It is not penalty-only, because the optimizer still receives explicit linear inequalities.

### C. Is the original nonlinear condition used?

Not directly inside the optimization problem.

The original nonlinear idea

`||p(k) - c_obs|| >= r_safe`

is not imposed as a nonlinear constraint in the solver.

### D. Is the professor-proposed linearized condition used?

Yes, in equivalent half-space form.

The code does not write the condition literally as

`n(k)' * (p(k) - c_obs) + s(k) >= r_safe`

but it does enforce the equivalent affine inequality

`n(k)' * p(k) + s(k) >= n(k)' * p_boundary`

where `p_boundary` is a point on the inflated obstacle or wall boundary. That boundary point already encodes the center and safe radius.

So conceptually the professor-proposed linearized soft constraint is implemented, but in a generalized signed-distance form that also supports:

- vertical cylindrical escape
- wall half-planes
- corner cases near vertical obstacle caps

### E. Is the slack variable actually part of the optimization variable?

Yes.

In `branch_solve_obstacle_deepc.m`, the optimizer solves over `[g; s_obs]`, not only `g`.

### F. Is the slack penalized in the cost?

Yes.

The code adds:

- `2 * rhoObs * I` in the slack block of the Hessian

so the slack is quadratically penalized.

### G. Are constraints applied over the prediction horizon or only evaluated after simulation?

They are applied over the prediction horizon.

`branch_build_obstacle_constraints.m` loops over `j = 1:T_f` and builds constraint rows for each prediction step.

### H. Are constraints applied to the predicted trajectory, the executed trajectory, or only post-processed?

They are applied to the predicted trajectory.

Specifically:

- `branch_build_obstacle_constraints.m` maps predicted DeePC outputs through `config.positionMap{j}`
- the solver constrains predicted positions
- only the first input is then applied to the plant
- executed safety is evaluated later by `branch_compute_safety_margin`

So:

- optimization target: predicted trajectory
- audit metric `h(k)`: executed trajectory

### I. Are obstacle and wall geometries consistent between constraint construction, safety-margin evaluation, and plotting?

Partially.

Obstacle geometry is consistent:

- constraints use `scenario.obstacles`
- safety evaluation uses `scenario.obstacles`
- plotting uses `scenario.obstacles`
- all use the same inflated safe radius `obs(5) + droneRadius + safeDistance`

Wall geometry is only partially consistent:

- constraint code can use walls through `labyrinthWallSegments(s)`
- safety evaluation uses walls
- plotting uses walls
- but the saved obstacle-aware DeePC runs set `IncludeWalls = false`

So the codebase supports wall constraints, but the reported DeePC obstacle-aware experiments did not actually enforce them in the optimization.

### J. Does the final solution satisfy `h(k) >= 0`?

No.

Saved branch results:

- soft DeePC: `min h = -0.098363 m`
- iterative DeePC: `min h = -0.098489 m`

### K. If not, why not?

The main reasons visible in the current implementation are:

1. The constraints are soft, so violation is explicitly allowed through slack.
2. The constraints are linearized locally around a nominal trajectory, so they are only an approximation of the nonlinear safety set.
3. Only the predicted trajectory is constrained; the executed trajectory can still deviate after the first control move is applied.
4. The current run configuration disables wall constraints with `IncludeWalls = false`.
5. The builder only keeps a limited number of active constraints per step with `MaxConstraintsPerStep = 2`.
6. Constraint activation is local, based on `ActivationDistance`.
7. The worst residual violation remains near obstacle `4`, indicating that the current nominal update and horizon are not strong enough to fully clear the last critical segment.

Bottom line for DeePC:

- the professor-style soft linearized obstacle constraints are implemented inside the DeePC optimization
- but the current branch realization is weak rather than fully effective

---

## MPC Audit

### A. Is obstacle avoidance included inside the optimization problem?

Yes, but only in the branch-side obstacle-aware wrapper.

Important distinction:

- copied TCC files such as `mpc_controller_mosek_v2.m` do not contain professor-style obstacle constraints
- the actual obstacle-aware MPC implementation lives in:
  - `branch/mpc_baseline/functions/mpc_solve_obstacle_aware.m`
  - `branch/mpc_baseline/functions/mpc_solve_tracking.m`

### B. Is it hard, soft, linearized, or only a cost penalty?

It is implemented as:

- soft constraint
- linearized constraint
- with a quadratic slack penalty

It is not only a cost penalty.

### C. Is the original nonlinear condition used?

Not directly inside the optimization problem.

The branch-side MPC does not solve the nonlinear condition

`||p(k) - c_obs|| >= r_safe`

as-is.

### D. Is the professor-proposed linearized condition used?

Yes, in equivalent affine form.

`mpc_solve_obstacle_aware.m` uses `branch_linearize_obstacle_constraints(...)` to obtain:

- normal vector `n(k)`
- boundary point on the inflated obstacle or wall

and then builds inequalities over the predicted state map:

`n(k)' * p_pred(k) + s(k) >= n(k)' * p_boundary`

That is the same professor-style soft linearization idea, rewritten with a boundary point instead of explicit `(c_obs, r_safe)`.

### E. Is the slack variable actually part of the optimization variable?

Yes.

`mpc_solve_tracking.m` augments the decision variable from `U` to `[U; s_obs]` whenever `extra.useSlack = true`.

### F. Is the slack penalized in the cost?

Yes.

`mpc_solve_tracking.m` adds a slack Hessian block:

- `H = blkdiag(H_u, 2 * rhoSlack * I)`

### G. Are constraints applied over the prediction horizon or only evaluated after simulation?

They are applied over the prediction horizon.

`mpc_solve_obstacle_aware.m` loops over `j = 1:N` and creates constraints for each future prediction step.

### H. Are constraints applied to the predicted trajectory, the executed trajectory, or only post-processed?

They are applied to the predicted trajectory.

Specifically:

- the optimizer constrains predicted positions through `Su` and `Sx`
- only the first control move is applied to the plant
- executed safety is evaluated afterward by `mpc_compute_safety_margin`

So the same prediction-versus-execution caveat applies here as in DeePC.

### I. Are obstacle and wall geometries consistent between constraint construction, safety-margin evaluation, and plotting?

Partially.

Obstacle geometry is consistent:

- constraints use `scenario.obstacles`
- safety evaluation reuses `branch_compute_safety_margin`
- 2D and 3D plotting use the same obstacle cylinders and same inflated safe radius

Wall geometry is only partially consistent:

- obstacle-aware MPC code can constrain walls
- safety evaluation and plotting include walls
- but the saved obstacle-aware MPC runs use `IncludeWalls = false`

So the same run-level inconsistency exists here too: walls are displayed and audited, but not actually enforced in the reported obstacle-aware MPC optimization.

### J. Does the final solution satisfy `h(k) >= 0`?

No.

Saved branch results:

- soft MPC: `min h = -0.016242 m`
- iterative MPC: `min h = -0.016185 m`

These are better than the current DeePC branch results, but still not strictly safe.

### K. If not, why not?

For the same structural reasons as DeePC:

1. The constraints are soft.
2. The constraints are only local linearizations.
3. The optimizer constrains the predicted trajectory, not the fully realized executed rollout.
4. The run configuration disables wall constraints with `IncludeWalls = false`.
5. The worst remaining violation still occurs at obstacle `4`.

Bottom line for MPC:

- the professor-style soft linearized obstacle constraints are implemented inside the branch MPC optimization
- they are more effective than the current branch DeePC version
- but they are still not strong enough to guarantee `h(k) >= 0`

---

## File-Level Conclusions

### DeePC files

- `branch_solve_obstacle_deepc.m`
  - obstacle constraints are inside the optimizer
  - slack is an optimization variable
  - slack is penalized
- `branch_build_obstacle_constraints.m`
  - builds horizon-wise linearized half-space constraints
  - supports both obstacles and walls
  - current runs only enabled obstacles
- `branch_linearize_obstacle_constraints.m`
  - computes local normals and boundary points from inflated signed distance
  - this is the main linearization engine
- `run_03_branch_soft_obstacle_deepc.m`
  - runs the soft constrained version
  - `IncludeWalls = false`
- `run_04_branch_iterative_obstacle_deepc.m`
  - repeats the solve with updated nominal predictions
  - still `IncludeWalls = false`

### MPC files

- `copied_from_tcc/mpc_controller_mosek_v2.m`
  - no obstacle constraints
  - no professor-style slack formulation
- `mpc_solve_tracking.m`
  - generic condensed QP
  - obstacle-aware only when called with extra inequalities and slack
- `mpc_solve_obstacle_aware.m`
  - actual branch-side obstacle-aware constraint insertion
- `mpc_run_labyrinth_rollout.m`
  - applies obstacle-aware solves in soft or iterative mode
- `run_03_mpc_labyrinth_obstacle_aware.m`
  - turns on obstacle constraints
  - `IncludeWalls = false`

---

## Direct Answers

### 1. Are the professor-proposed constraints implemented in DeePC?

Yes, but in equivalent generalized half-space form rather than in the exact literal notation.

They are:

- inside the DeePC optimization
- linearized
- soft
- slack-based
- horizon-wise

They are not:

- exact nonlinear norm constraints
- hard safety guarantees
- fully effective in the current saved runs

### 2. Are the professor-proposed constraints implemented in MPC?

Yes, in the branch-side MPC wrappers.

They are not present in the original copied TCC controller by themselves, but they are implemented in:

- `mpc_solve_obstacle_aware.m`
- `mpc_solve_tracking.m`

with the same soft linearized slack-based logic.

### 3. Which controller actually enforces obstacle safety better?

The current branch MPC implementation.

Based on the saved runs:

- DeePC iterative: `min h = -0.098489 m`, `11` safety violations
- MPC iterative: `min h = -0.016185 m`, `1` safety violation

So MPC enforces the current branch obstacle safety better, even though it still does not achieve full `h(k) >= 0`.

### 4. Which claims are safe to write in the article?

Safe claims:

- both branch controllers include obstacle-aware constraints inside the optimization problem
- in both cases the obstacle-aware formulation is soft, linearized, and slack-based
- the current implementation constrains predicted trajectories over the horizon
- the current branch MPC obstacle-aware controller reduced safety violations more than the current branch DeePC obstacle-aware controller in the tested labyrinth scenario
- neither controller yet guarantees strictly nonnegative safety margin along the entire executed trajectory

### 5. Which claims must be avoided?

Avoid these claims:

- that the exact nonlinear obstacle constraint is solved directly
- that the reported obstacle constraints are hard safety constraints
- that the current branch results are collision-free or guarantee `h(k) >= 0`
- that wall constraints were fully enforced in the reported obstacle-aware runs
- that DeePC currently outperforms MPC on obstacle safety in this branch
- that the current branch implementation already proves final obstacle-avoidance success
