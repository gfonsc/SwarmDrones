# Branch Implementation Plan

## Purpose

This plan defines how the obstacle-aware DeePC branch should be built while keeping the original `SwarmDrones` project read-only.

All new work must stay inside:

- `C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch`

## 1. Which parts of the original demo will be reused

The branch should preserve the original labyrinth demo as the reference implementation and reuse these read-only components:

- `examples/demo_deepc_labyrinth_database.m`
  - as the behavioral baseline and audit target;
- `database/loadLabyrinthData.m`
  - for the canonical labyrinth training data;
- `database/labyrinthScenarioConfig.m`
  - for walls, obstacles, bounds, mission setpoints, and model parameters;
- `database/labyrinthWallSegments.m`
  - for wall geometry extraction;
- `database/labyrinthMinWallClearance.m`
  - for baseline wall-clearance validation;
- `models/simulateLabyrinthDroneStep.m`
  - for plant rollout;
- `controllers/deepc/deepcSetup.m`
  - for the original DeePC offline build;
- `controllers/deepc/deepcAlgorithm.m`
  - for baseline replay when no obstacle-aware augmentation is needed;
- `controllers/deepc/functions/solveQP.m`
  - as the generic QP backend for branch-side obstacle-aware solves.

## 2. Which original functions will be called read-only

The branch should call the original files read-only, not copy them unless absolutely necessary:

- `loadLabyrinthData`
- `deepcSetup`
- `deepcAlgorithm`
- `simulateLabyrinthDroneStep`
- `labyrinthScenarioConfig`
- `labyrinthWallSegments`
- `labyrinthMinWallClearance`
- optionally the original database generator if regeneration is needed

The branch should not modify:

- original examples;
- original controllers;
- original database scripts;
- original model files;
- original results directories.

## 3. Which new branch functions will be created later

The branch should add only the minimal new files needed to extend behavior cleanly:

- `branch_extract_demo_config`
  - extract the original labyrinth scenario and baseline DeePC parameters into a branch-controlled struct;
- `branch_compute_safety_margin`
  - compute obstacle and wall clearance and define `h(k)`;
- `branch_build_obstacle_constraints`
  - build obstacle-aware linear inequalities over the prediction horizon;
- `branch_linearize_obstacle_constraints`
  - generate successive linearizations around a nominal predicted path;
- `branch_solve_obstacle_deepc`
  - branch-side wrapper that augments the original DeePC QP with slack variables and obstacle constraints;
- `branch_plot_labyrinth_results`
  - branch-side plotting utility that saves only under `branch/`.

These functions should reuse the original scenario and solver interfaces as much as possible instead of replacing them.

## 4. How the original labyrinth scenario will be preserved

The branch should preserve the original labyrinth scenario in four ways:

1. Same canonical geometry source
   - read `labyrinthScenarioConfig()` directly from the original project.
2. Same canonical training data source
   - load the labyrinth DeePC database through `loadLabyrinthData`.
3. Same original plant surrogate
   - use `simulateLabyrinthDroneStep` for baseline and branch rollouts.
4. Same baseline DeePC structure
   - retain the original normalization, horizon definitions, and DeePC model build unless a branch experiment explicitly varies them.

The first branch baseline should mirror the original demo as closely as possible before obstacle-aware modifications are added.

## 5. How the professor's improvement will be added

The professor's suggested improvement is:

- obstacle-aware DeePC constraints;
- linearization of nonconvex safety constraints;
- iterative updates over the prediction horizon.

The clean branch strategy is:

1. Keep the original DeePC data model and solver backend.
2. Add new branch-side obstacle constraints without editing the original solver files.
3. Build a branch-side wrapper around the original QP backend.
4. Introduce slack variables for obstacle-constraint softness.
5. Add successive linearization only after the baseline and soft version are working.

This keeps the branch aligned with the original project while still implementing the article's new methodological contribution.

## 6. Constraint strategy to implement

The branch should support three related formulations, in this order:

### Baseline

Original DeePC only:

- no obstacle constraints in the QP;
- used to reproduce the labyrinth reference behavior and establish baseline metrics.

### Soft obstacle-aware DeePC

Preferred first obstacle-aware version:

- linearized obstacle or wall constraints over the horizon;
- slack variables `s >= 0`;
- obstacle penalty term such as `rho_obs * ||s||^2`.

This is the lowest-risk extension because it preserves feasibility more gracefully than hard nonconvex constraints.

### Iterative obstacle-aware DeePC

Successive-linearization extension:

- initialize a nominal predicted path `p_nom`;
- linearize obstacle constraints around `p_nom`;
- solve the augmented QP;
- update `p_nom`;
- repeat for a small number of iterations.

This version most closely matches the professor's suggestion for handling nonconvex safety constraints.

## 7. Which implementation will be done first

Recommended order:

### Step A

Replay or reproduce the original labyrinth demo from `branch/` without modifying the original file.

Success condition:

- branch-side script can run the same scenario logic;
- results are saved only inside `branch/`;
- original dependencies load correctly.

### Step B

Create a branch baseline using the same scenario and original DeePC behavior.

Success condition:

- baseline metrics are computed in `branch/results`;
- baseline figures are saved in `branch/figures/baseline`;
- baseline behavior is meaningfully comparable to the original demo.

### Step C

Add branch safety-margin computation and safety plots.

Success condition:

- branch can compute obstacle clearance, wall clearance, and signed or approximate safety margin;
- violations can be counted consistently.

### Step D

Add a soft obstacle-aware DeePC version.

Success condition:

- branch-side QP includes obstacle-aware linear inequalities or a compatible approximation;
- slack values are reported;
- obstacle metrics improve relative to the baseline where needed.

### Step E

Add iterative successive linearization.

Success condition:

- branch updates obstacle linearization around a nominal predicted trajectory;
- convergence and per-iteration behavior are logged;
- results can be compared with the soft single-pass version.

### Step F

Compare baseline, soft obstacle-aware, and iterative obstacle-aware variants.

Success condition:

- summary metrics table exists;
- paper-ready figures exist if runs succeed;
- claims are separated into supported versus not-yet-supported.

## 8. What will be considered success

This branch stage should be considered successful if:

1. No original file outside `branch/` is modified.
2. The original labyrinth demo is understood and documented.
3. A branch replay path exists without writing into original folders.
4. A branch baseline reproduces the same scenario structure.
5. Safety margin can be computed against both walls and obstacles.
6. A soft obstacle-aware DeePC attempt runs through a branch wrapper.
7. An iterative obstacle-aware DeePC attempt runs through a branch wrapper.
8. Metrics, figures, and reports are saved only under `branch/`.

## 9. Technical design choice for the solver wrapper

The current DeePC API is convenient for the baseline but too rigid for obstacle-aware augmentation because:

- `deepcSetup` precomputes fixed `Aineq` and `bIneq`;
- `deepcAlgorithm` expects a `g`-only decision vector;
- `extractSolution` assumes no extra slack variables exist.

Therefore the branch should:

1. Reuse the original DeePC setup to obtain:
   - `Up`, `Uf`, `Yp`, `Yf`;
   - baseline `H`;
   - baseline bound constraints.
2. Build a branch-side augmented QP of the form:
   - decision variable `[g; s]`;
   - branch-side Hessian and linear cost;
   - original equalities plus added obstacle inequalities.
3. Reuse `solveQP` directly to solve the augmented problem.
4. Perform branch-side solution extraction for `g`, `u`, `y`, and slack.

This approach respects the read-only rule and minimizes divergence from the original architecture.

## 10. Geometry strategy for the branch

The original labyrinth does not consist only of circles. It contains:

- explicit wall segments;
- vertically bounded cylindrical obstacles.

So the branch should avoid forcing a circle-only formulation.

Recommended geometry handling:

- wall constraints:
  - use distance to wall segments and, where practical, local half-plane approximations;
- cylindrical obstacle constraints:
  - use XY radial distance with altitude activation over the obstacle's `[z_min, z_max]` band;
- safety metric:
  - define `h(k)` as the minimum of relevant clearances minus the required safety buffer.

If a precise signed-distance model becomes too heavy for the first branch pass, a documented local linearization around the closest wall or obstacle is acceptable.

## 11. Immediate deliverable for the next phase

The next implementation phase should not start by editing control logic directly.

It should start with:

1. branch-side setup script;
2. branch-side replay script;
3. branch-side baseline runner;
4. branch-side config extractor;
5. branch-side safety-margin utility.

Only after the baseline is reproducible should the obstacle-aware QP wrapper be introduced.

## Key conclusion

The safest and cleanest branch path is:

- preserve the original labyrinth example as the baseline reference;
- reuse the original data, geometry, model, and DeePC setup read-only;
- add the professor's improvement only through branch-side wrappers and utilities;
- introduce obstacle awareness first as a soft, slack-enabled branch formulation;
- then add iterative successive linearization on top of that branch formulation.
