# Dependency Map

## Scope

This map covers:

- direct dependencies of `examples/demo_deepc_labyrinth_database.m`;
- the most important second-level dependencies that shape the DeePC model, solver, data interface, and obstacle geometry;
- branch implications for reuse versus wrapper creation.

An expanded inventory is also saved in:

- `C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch\tables\dependency_inventory.csv`

## A. Direct call graph from the original demo

### External read-only functions called directly

| Function | File path | Signature | Category | Main role | Reuse mode | Branch wrapper needed |
|---|---|---|---|---|---|---|
| `loadLabyrinthData` | `database/loadLabyrinthData.m` | `[U, Y, t, meta] = loadLabyrinthData(varargin)` | database | Load or regenerate the labyrinth DeePC training database | direct read-only reuse | no |
| `deepcSetup` | `controllers/deepc/deepcSetup.m` | `model = deepcSetup(uData, yData, params)` | DeePC offline | Build Hankel matrices, Hessian, and fixed constraints | direct read-only reuse | no |
| `deepcAlgorithm` | `controllers/deepc/deepcAlgorithm.m` | `[uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r, solverOpts)` | DeePC online | Solve the standard DeePC QP each control step | direct read-only reuse for baseline | yes for obstacle-aware extension |
| `simulateLabyrinthDroneStep` | `models/simulateLabyrinthDroneStep.m` | `[stateNext, y, uClipped] = simulateLabyrinthDroneStep(state, u, cfg)` | dynamics/model | Simulate the original kinematic labyrinth drone | direct read-only reuse | no |
| `labyrinthMinWallClearance` | `database/labyrinthMinWallClearance.m` | `[clearance, sampleIdx, wallIdx] = labyrinthMinWallClearance(traj, s)` | obstacle/wall geometry | Compute minimum wall clearance for the trajectory | direct read-only reuse | no |

### Internal local helper functions inside the original script

These are local functions defined inside `demo_deepc_labyrinth_database.m` itself.

| Local helper | Main role |
|---|---|
| `selectOutput` | Extract the modeled output channels from state/output vectors |
| `initLog`, `trimLog` | Allocate and finalize logging |
| `advancePathIndex` | Advance along the dense reference path |
| `buildReferenceHorizon` | Create the stacked future reference vector |
| `pathVelocity` | Estimate reference velocity for horizon construction |
| `denormalizePrediction` | Convert normalized DeePC predictions back to physical units |
| `buildResult` | Pack all outputs into a result struct |
| `computeMetrics` | Compute RMSE, clearance, bypass, and safety metrics |
| `validateResult` | Enforce success assertions |
| `plotLabyrinth2D`, `plotLabyrinth3D` | Plot final mission results |
| `drawCorridorWalls2D`, `drawCorridorWalls3D`, `drawObstacles2D`, `drawObstacles3D`, `drawCeiling` | Environment rendering |
| `saveFigure` | Save figures |
| `resamplePolyline3D` | Densify the mission path |

Branch implication:

These helpers are not reusable as standalone functions from the original file. If the branch needs them, it should either:

- reproduce only the needed logic in new branch files; or
- create branch-specific extraction or plotting helpers with the same intent, not by editing the original script.

## B. Important second-level dependencies

### DeePC stack

| Function | File path | Signature | Notes |
|---|---|---|---|
| `validateParams` | `controllers/deepc/functions/validateParams.m` | `params = validateParams(params, m, p)` | Expands per-step `Q`, `R`, `u/y` bounds to full horizon and validates shapes |
| `buildHankelMatrix` | `controllers/deepc/functions/buildHankelMatrix.m` | `[Up, Yp, Uf, Yf] = buildHankelMatrix(uData, yData, T_ini, T_f)` | Expected data shapes are `uData: m x T`, `yData: p x T` |
| `checkPersistency` | `controllers/deepc/functions/checkPersistency.m` | called by `deepcSetup` | Persistency of excitation check |
| `computeSteadyStateG` | `controllers/deepc/functions/computeSteadyStateG.m` | `gRef = computeSteadyStateG(Up, Yp, Uf, Yf, params)` | Computes `g_r` for regularization |
| `buildHessian` | `controllers/deepc/functions/buildHessian.m` | `H = buildHessian(Uf, Yf, Yp, nG, params)` | Builds Hessian for decision variable `g` only |
| `buildConstraintMatrix` | `controllers/deepc/functions/buildConstraintMatrix.m` | `[Aeq, Aineq, bIneq] = buildConstraintMatrix(Up, Uf, Yf, params)` | Supports equality plus linear inequality box constraints |
| `buildLinearCost` | `controllers/deepc/functions/buildLinearCost.m` | `f = buildLinearCost(precomp, r, yIni)` | Online varying term |
| `solveQP` | `controllers/deepc/functions/solveQP.m` | `[zOpt, fval, exitflag, solverInfo] = solveQP(H, f, Aeq, beq, Aineq, bIneq, solverOpts)` | Generic QP backend; strongest reuse point for a branch wrapper |
| `extractSolution` | `controllers/deepc/functions/extractSolution.m` | `[uOpt, yPred, info] = extractSolution(zOpt, Uf, Yf, fval, exitflag, solverInfo, params)` | Assumes the decision variable corresponds to `g` only |

Branch implication:

- `solveQP` is generic enough to be reused directly from the branch.
- `deepcAlgorithm` and `extractSolution` are tailored to the original decision vector `g`.
- If obstacle slack variables are added, a branch-side wrapper or branch-side solver function will be needed to handle an augmented decision variable such as `[g; s]`.

### Database and geometry stack

| Function | File path | Signature | Notes |
|---|---|---|---|
| `generateLabyrinthDeePCDatabase` | `database/generateLabyrinthDeePCDatabase.m` | `dataFile = generateLabyrinthDeePCDatabase(varargin)` | Regenerates the canonical training dataset |
| `labyrinthScenarioConfig` | `database/labyrinthScenarioConfig.m` | `s = labyrinthScenarioConfig()` | Main source of walls, obstacles, bounds, mission setpoints, and model config |
| `labyrinthWallSegments` | `database/labyrinthWallSegments.m` | `wallSegments = labyrinthWallSegments(s)` | Returns explicit walls or derives them from the corridor |
| `labyrinthMinWallClearance` | `database/labyrinthMinWallClearance.m` | `[clearance, sampleIdx, wallIdx] = labyrinthMinWallClearance(traj, s)` | Post-run wall clearance audit |

Branch implication:

- the branch should treat `labyrinthScenarioConfig` as the ground-truth scenario definition;
- wall-aware safety calculations should reuse `labyrinthWallSegments` and `labyrinthMinWallClearance` rather than replacing the geometry model.

### Nearby obstacle-planner utilities

These are not used directly by the audited demo, but they are relevant context because they already represent obstacle geometry and avoidance behavior elsewhere in the project.

| Function | File path | Signature | Category | Relevance |
|---|---|---|---|---|
| `pathPlannerSetup` | `path_planner/pathPlannerSetup.m` | `planner = pathPlannerSetup(goal, params)` | planner | Initializes wall-follower avoidance FSM |
| `pathPlannerStep` | `path_planner/pathPlannerStep.m` | `[setpoint, yaw_cmd, planner, info] = pathPlannerStep(planner, pos, yaw, obstacles)` | planner | Produces obstacle-aware setpoints, but outside DeePC |
| `checkCollision` | `path_planner/functions/checkCollision.m` | `[collision, obsIdx] = checkCollision(pos, obstacles, droneRadius)` | obstacle utility | Useful reference for obstacle-body geometry |
| `detectObstacles` | `path_planner/functions/detectObstacles.m` | called by `pathPlannerStep` | obstacle utility | Useful as a geometry/FOV reference, not as DeePC constraint logic |
| `plotAvoidanceResults` | `path_planner/functions/plotAvoidanceResults.m` | plotting utility | plotting | Reusable plotting ideas for branch figures |

Branch implication:

These planner files can inform the branch's obstacle representation and safety plotting, but they are not a substitute for obstacle-aware constraints inside the DeePC QP.

## C. Data interfaces and expected shapes

### Original labyrinth training data

From `generateLabyrinthDeePCDatabase` and the audited demo:

- `U_train`: `4 x T`
  - channels: `vx_cmd`, `vy_cmd`, `vz_cmd`, `yaw_rate_cmd`
- `Y_train`: `6 x T`
  - channels: `x`, `y`, `z`, `vx`, `vy`, `vz`
- full simulated state `X`: `8 x T`
  - `[x y z vx vy vz yaw yawRate]'`

### DeePC horizon shapes in the original demo

With `m = 4`, `p = 6`, `T_ini = 4`, `T_f = 8`:

- `uIni`: `(m * T_ini) x 1 = 16 x 1`
- `yIni`: `(p * T_ini) x 1 = 24 x 1`
- `r`: `(p * T_f) x 1 = 48 x 1`
- `uOpt`: `(m * T_f) x 1 = 32 x 1`
- `yPred`: `(p * T_f) x 1 = 48 x 1`

### Scenario geometry data

From `labyrinthScenarioConfig`:

- obstacle rows: `[x, y, z_min, z_max, radius, mode_id]`
- wall rows: `[x1, y1, x2, y2]`

Branch implication:

The obstacle-aware branch should not assume the environment is "circles only". The original scenario is wall-and-cylinder based, and the safety metric should support both.

## D. Solver and constraint capability audit

### What the current solver already supports

The existing DeePC solver chain already supports:

- fixed linear equality constraints;
- fixed linear inequality constraints;
- input bounds;
- output bounds;
- MOSEK or `quadprog` as QP backends.

### What it does not directly support yet

The current public DeePC interface does not directly support:

- time-varying obstacle inequalities added online;
- slack variables appended to the decision vector;
- iterative successive linearization within one control step;
- nonlinear signed-distance or quadratic obstacle constraints;
- YALMIP, CVX, or CasADi.

Search result summary:

- no dedicated MPC implementation was found in this pass;
- no YALMIP, CVX, or CasADi usage was found;
- the QP backends are MOSEK and MATLAB `quadprog`.

## E. Reuse policy by component

| Component | Reuse decision | Reason |
|---|---|---|
| `loadLabyrinthData` | reuse directly | canonical labyrinth dataset entry point |
| `labyrinthScenarioConfig` | reuse directly | canonical static geometry and mission definition |
| `simulateLabyrinthDroneStep` | reuse directly | keeps branch dynamics aligned with the original demo |
| `deepcSetup` | reuse directly | preserves the original DeePC formulation for baseline |
| `deepcAlgorithm` | reuse for baseline only | current interface does not expose online obstacle constraints |
| `solveQP` | reuse directly in branch wrapper | generic solver backend already supports linear inequalities |
| plotting logic in original script | branch reimplementation | original plotting functions are local to the script |
| result saving logic in original script | branch reimplementation | original script writes to read-only original folders |

## F. Recommended branch wrappers

To respect the read-only rule while still extending behavior, the branch will likely need these wrapper roles later:

1. A branch config extractor
   - to capture the original scenario, scaling, horizon, and constraints cleanly.
2. A branch baseline runner
   - to reproduce original behavior while saving only into `branch/`.
3. A branch obstacle-constraint builder
   - to map wall or obstacle geometry into linearized safety inequalities over the horizon.
4. A branch DeePC QP wrapper
   - to augment the original `g`-only QP with slack variables and time-varying inequalities while still reusing `solveQP`.
5. A branch safety-margin utility
   - to compute minimum clearance and violation statistics consistently for baseline and obstacle-aware runs.

## Key conclusion

The original demo already provides a strong base layer:

- scenario;
- database;
- DeePC setup;
- closed-loop rollout;
- solver backend;
- geometry auditing.

The main missing interface for the professor's improvement is not "how to solve a QP" but "how to inject obstacle-aware, time-varying, slack-enabled constraints into the existing DeePC loop without editing the original files."
