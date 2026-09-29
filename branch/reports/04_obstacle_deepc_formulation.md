# Obstacle DeePC Formulation

## Goal

The branch extends the original read-only labyrinth DeePC example with obstacle-aware predictive constraints, while preserving:

- the original labyrinth geometry;
- the original training database;
- the original DeePC offline setup;
- the original kinematic plant surrogate;
- the original QP backend.

## Original environment representation

The original labyrinth scenario uses two kinds of collision geometry:

1. Cylindrical obstacles
   - stored as rows `[x, y, z_min, z_max, radius, mode_id]`.
2. Wall segments
   - stored as rows `[x1, y1, x2, y2]`.

The original demo did not enforce these geometries directly inside the DeePC QP. It mainly:

- tracked a pre-shaped safe path;
- enforced global input and output bounds;
- checked obstacle and wall clearance after the rollout.

## Branch safety metric

The branch defines the safety margin

`h(k) = d_safe(k)`

where `d_safe(k)` is the signed distance from the drone position to the nearest unsafe set.

Unsafe-set definition used in the branch:

- for obstacles:
  - the unsafe set is the original cylinder inflated by `droneRadius + safeDistance` in the radial direction;
- for walls:
  - the unsafe set is the wall segment offset by `droneRadius + safeDistance` in the local normal direction.

Interpretation:

- `h(k) > 0`: outside the unsafe set;
- `h(k) = 0`: on the safety boundary;
- `h(k) < 0`: inside the safety boundary.

## Branch optimization strategy

The branch uses the original DeePC cost on `g` and augments the online QP with branch-side slack variables:

`z = [g; s]`

The cost becomes:

- original DeePC quadratic cost in `g`;
- plus `rho_obs * ||s||^2`.

The branch does not modify the original solver files. Instead it:

1. reuses the original offline model from `deepcSetup`;
2. rebuilds only the online augmented QP inside the branch;
3. reuses the original generic `solveQP` backend.

## Linearized safety constraint

For one local feature and one nominal point `p_nom`, the branch computes:

- a unit outward normal `n`;
- a boundary point `q` on the inflated unsafe-set boundary.

The linearized constraint is:

`n' * (p - q) + s >= 0`

which is passed to the QP in equivalent linear-inequality form.

This is a local half-space approximation of the nonconvex obstacle-avoidance condition.

## Soft formulation

The soft branch solver:

- builds linearized obstacle constraints along the prediction horizon;
- adds one nonnegative slack variable per active linearized constraint;
- penalizes slack quadratically.

Saved default branch configuration:

- `rho_obs = 1e5`
- `ActivationDistance = 1.0`
- `MaxConstraintsPerStep = 2`

## Iterative formulation

The iterative branch solver performs successive linearization:

1. initialize the nominal future path from the shifted previous prediction or the current reference horizon;
2. build linearized obstacle constraints;
3. solve the augmented DeePC QP;
4. update the nominal path with the predicted trajectory;
5. repeat for up to `MaxIter = 4` or until the residual is small.

## Important branch design choice

During branch tuning, two configurations were tested:

1. wall-and-obstacle constraints together;
2. obstacle-only constraints, while still monitoring wall margins in the reports.

The saved branch default uses the obstacle-only constraint set because:

- it improved the worst obstacle margin slightly;
- the combined wall-plus-obstacle version tended to steer the trajectory into obstacle 4 in the last serpentine segment.

So the current saved branch prototype is:

- obstacle-constrained;
- wall-monitored;
- not yet a full wall-constrained final controller.

## Assumptions and limitations

1. The current branch keeps the original horizon `T_f = 8`.
2. The current branch keeps the original DeePC input/output interface:
   - `u = [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`
   - `y = [x, y, z, vx, vy, vz]`
3. The current branch uses local linearizations, not exact nonlinear obstacle constraints.
4. The current branch does not yet prove strict safety satisfaction in the entire labyrinth.
5. The current branch does not compare against MPC yet.

## Practical conclusion

The branch now contains a valid professor-aligned prototype:

- DeePC with explicit obstacle-aware predictive constraints;
- quadratic slack penalization;
- an iterative successive-linearization option;
- all implemented without changing the original project files.

The next work should focus on making these constraints materially stronger, especially around obstacle 4, likely through:

- a longer horizon;
- richer obstacle activation logic;
- obstacle-specific or wall-specific weighting;
- or a better nominal-path initialization.
