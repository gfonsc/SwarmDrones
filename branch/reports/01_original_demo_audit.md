# Original Labyrinth Demo Audit

## Scope

This audit covers the original read-only file:

- `C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\examples\demo_deepc_labyrinth_database.m`

It also references the key read-only dependencies that this script calls from `controllers/deepc`, `database`, and `models`.

## 1. What `demo_deepc_labyrinth_database.m` does

The file is a complete single-drone DeePC closed-loop mission demo for a handcrafted labyrinth environment. It:

1. Adds the original DeePC, database, and model folders to the MATLAB path.
2. Loads or regenerates a labyrinth DeePC database with `loadLabyrinthData('Regenerate', true, 'NumSamples', 360, 'Seed', 11)`.
3. Extracts the labyrinth scenario and kinematic drone model from the database metadata.
4. Normalizes the training input/output data.
5. Builds an offline DeePC model with `deepcSetup`.
6. Creates a dense 3D reference path from mission setpoints.
7. Runs a receding-horizon closed-loop simulation with `deepcAlgorithm`.
8. Simulates the drone with `simulateLabyrinthDroneStep`.
9. Logs tracking, prediction, solver success, and trajectory metrics.
10. Validates the result with hard assertions.
11. Plots 2D and 3D figures and saves outputs under the original `examples/results` folder.

## 2. Whether it is a script or function

`demo_deepc_labyrinth_database.m` is a MATLAB script, not a top-level function.

It contains many local helper functions at the end of the file, but the main entry point is script-style and executes immediately when run.

## 3. What scenario it creates

The scenario is a static labyrinth mission defined by `labyrinthScenarioConfig()` and carried inside the database metadata.

The mission starts at `A = [0, 0, 0]`, climbs to hover altitude, and tracks a serpentine corridor path toward `B`, moving through a maze-like environment with:

- corridor walls;
- four cylindrical obstacles;
- vertical constraints through floor and ceiling bounds;
- a reference path that intentionally includes both lateral bypasses and altitude changes.

The online mission path comes from `scenario.trainingSetpoints` when present, otherwise `scenario.setpoints`, then gets densified by the script's local `resamplePolyline3D`.

## 4. Whether it uses one drone or multiple drones

It is a single-drone demo.

Evidence:

- the simulated state is one 8-state vector;
- the input is one 4-channel command vector;
- the plant update uses one call to `simulateLabyrinthDroneStep`;
- no swarm coordination or multi-agent coupling appears in the script.

The repository name is `SwarmDrones`, but this specific labyrinth example is not a swarm controller.

## 5. What the labyrinth or obstacle environment looks like

The original environment is static and mixed:

- physical wall segments define the maze corridor;
- four cylindrical obstacles define localized avoidance challenges;
- a ceiling limit and floor limit constrain altitude.

From `labyrinthScenarioConfig()`:

- walls are explicit 2D line segments in `s.wallSegments`;
- obstacles are stored as cylinders `[x, y, z_min, z_max, radius, mode_id]`;
- `mode_id` labels the intended maneuver style:
  - `1`: lateral right;
  - `2`: altitude up;
  - `3`: altitude down;
  - `4`: lateral left.

So the geometry is richer than simple circles: it includes walls plus vertically bounded cylindrical bodies.

## 6. What data or database it uses

The demo uses the offline labyrinth DeePC database returned by `loadLabyrinthData`.

That loader:

- reads `database/labyrinth/labyrinth_deepc_data.mat`, or
- regenerates it with `generateLabyrinthDeePCDatabase(...)` when `Regenerate=true`.

The generated dataset contains:

- `U`: applied command inputs;
- `Y`: measured outputs;
- `X`: full simulated state history;
- `t`: timestamps;
- `scenario`: embedded scenario definition;
- metadata such as names, sample count, and minimum obstacle clearance.

The excitation database is trajectory-level, not raw motor data. It uses a kinematic model with velocity-command inputs.

## 7. What variables are used as inputs `u`

The DeePC input is 4-dimensional and comes from `scenario.inputNames`:

- `vx_cmd`
- `vy_cmd`
- `vz_cmd`
- `yaw_rate_cmd`

So the original labyrinth demo does not use raw motor speeds. It uses commanded body-frame or inertial-frame velocity-like channels plus yaw-rate command, depending on how the kinematic model is interpreted.

Expected shape:

- training input `U_train`: `m x T` with `m = 4`;
- online past input buffer `uHist`: `m x T_ini`;
- DeePC optimal input trajectory `uOpt`: `(m * T_f) x 1`.

## 8. What variables are used as outputs `y`

The DeePC output is 6-dimensional and comes from `scenario.outputNames` with `scenario.outputIdx = 1:6`:

- `x`
- `y`
- `z`
- `vx`
- `vy`
- `vz`

The demo tracks a stacked output reference of dimension `p = 6`, even though the path itself is defined in position space and the velocity components are implicitly referenced to zero.

Expected shape:

- training output `Y_train`: `p x T` with `p = 6`;
- online past output buffer `yHist`: `p x T_ini`;
- DeePC predicted output `yPred`: `(p * T_f) x 1`.

## 9. What DeePC formulation is currently implemented

The current formulation is standard regularized DeePC with:

- hard past-input consistency `U_p g = u_ini`;
- soft past-output matching through `lambda_y ||Y_p g - y_ini||^2`;
- quadratic output tracking;
- quadratic input regularization;
- quadratic `g` regularization toward `g_r`;
- box constraints on future inputs and outputs.

The online optimizer solves:

`min_g (Y_f g - r)'Q(Y_f g - r) + (U_f g - u_r)'R(U_f g - u_r) + lambda_y ||Y_p g - y_ini||^2 + lambda_g ||g - g_r||^2`

subject to:

- `U_p g = u_ini`
- `u_min <= U_f g <= u_max`
- `y_min <= Y_f g <= y_max`

The script uses:

- `T_ini = 4`
- `T_f = 8`
- `lambda_y = 700`
- `lambda_g = 12`
- `qNorm = 2`

This is the original labyrinth configuration, not the previously validated `A_motors` dataset configuration from the other project branch.

## 10. What cost function is currently implemented

The cost is fully quadratic and assembled by the DeePC stack:

- output tracking cost through `Q = diag([850, 850, 980, 8, 8, 10])`;
- input penalty through `R = diag([0.018, 0.018, 0.030, 0.10])`;
- initial-output mismatch penalty with `lambda_y = 700`;
- `g` regularization with `lambda_g = 12`.

The output tracking weight emphasizes position more heavily than velocity, especially altitude.

There is no explicit obstacle-distance penalty term inside the current cost.

## 11. What constraints are currently implemented

Inside the DeePC QP, the currently implemented constraints are:

- one block of equality constraints `U_p g = u_ini`;
- future input lower and upper bounds through `U_f g`;
- future output lower and upper bounds through `Y_f g`.

Outside the QP, additional safety-related limits appear in the simulation and validation layers:

- state clamping inside `simulateLabyrinthDroneStep`;
- post-run minimum obstacle clearance checks;
- post-run wall clearance checks via `labyrinthMinWallClearance`;
- post-run assertions for ceiling and floor respect;
- post-run collision-free assertions.

So the safety geometry is enforced mainly through scenario design and validation, not through explicit obstacle inequalities in the DeePC optimizer.

## 12. Whether obstacle avoidance is hard-coded, constrained, penalized, or only visual

Obstacle avoidance in this original demo is not implemented as explicit obstacle constraints in the DeePC QP.

It is partly hard-coded at the scenario/path level and partly checked afterward:

- the reference mission path is already shaped to go around obstacles and through the maze;
- the output bounds keep the predicted motion inside a global operating envelope;
- the final trajectory is validated against obstacles and walls;
- the plotting functions visualize the maze and obstacles.

So obstacle handling is currently:

- not hard-constrained inside the DeePC optimization;
- not penalized in the DeePC objective;
- not iterative;
- not merely cosmetic, because collisions are audited and asserted afterward.

## 13. Whether the current obstacles are static or dynamic

The obstacles in this demo are static.

The geometry comes from fixed obstacle rows in `labyrinthScenarioConfig()` and fixed wall segments.

The repository also contains dynamic-labyrinth files such as:

- `examples/demo_deepc_labyrinth_dynamic_database.m`
- `database/loadLabyrinthDynamicData.m`
- `database/generateLabyrinthDynamicDeePCDatabase.m`

but those are separate from this audited script.

## 14. Whether the current solver supports inequality constraints

Yes.

The solver stack supports:

- equality constraints `Aeq * x = beq`;
- inequality constraints `Aineq * x <= bIneq`.

`buildConstraintMatrix` constructs the standard DeePC box constraints, and `solveQP` passes them to either:

- MOSEK, if available; or
- MATLAB `quadprog`, otherwise.

Important limitation:

The current `deepcAlgorithm` interface is built around a precomputed model whose inequalities are fixed at offline setup time. It does not currently expose a direct hook for time-varying obstacle inequalities per control step.

## 15. Whether slack variables are already used

No explicit obstacle slack variables are used in the current labyrinth DeePC demo.

Also, the base DeePC implementation does not augment the decision variable with extra slack variables for:

- obstacle violation;
- wall clearance violation;
- soft output bounds.

The only "softness" already present is the standard DeePC soft penalty on past-output consistency through `lambda_y`.

## 16. Whether an iterative procedure already exists

No iterative successive-linearization loop exists in `demo_deepc_labyrinth_database.m`.

The current online loop performs exactly one DeePC QP solve per control step and then applies the first control move.

There is no update of:

- a nominal predicted trajectory `p_nom`;
- obstacle normals `n(k)`;
- linearized safety half-spaces;
- repeated QP solves within the same control step.

## 17. What should be reused

The following parts are strong reuse candidates and should remain read-only:

- `database/labyrinthScenarioConfig.m`
  - original mission geometry, walls, cylinders, operating bounds.
- `database/loadLabyrinthData.m`
  - canonical loader for the labyrinth DeePC database.
- `database/generateLabyrinthDeePCDatabase.m`
  - canonical way to regenerate the original database when needed.
- `models/simulateLabyrinthDroneStep.m`
  - original plant surrogate used by this demo.
- `controllers/deepc/deepcSetup.m`
  - offline DeePC model construction.
- `controllers/deepc/deepcAlgorithm.m`
  - baseline online DeePC solve path.
- `controllers/deepc/functions/solveQP.m`
  - reusable generic QP backend for a branch wrapper.
- `database/labyrinthWallSegments.m`
  - canonical wall geometry extraction.
- `database/labyrinthMinWallClearance.m`
  - original wall-clearance audit utility.
- the plotting geometry conventions embedded in the original script
  - useful as the visual baseline for branch figures.

## 18. What should not be touched

Per the branching rules, the following must remain untouched:

- the original `examples/demo_deepc_labyrinth_database.m`;
- all original files under `controllers/`, `database/`, `models/`, `examples/`, `utils/`, and any other non-branch folder;
- the original data files under `database/labyrinth/`;
- the original results location under `examples/results`.

Even if the original demo needs adaptation, the adaptation must happen only through new files under:

- `C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch`

## 19. What is missing to match the professor's suggested improvement

To align this demo with the professor's suggested obstacle-aware DeePC direction, the branch still needs:

1. A branch replay path that reproduces the original demo behavior without writing into the original `examples/results`.
2. A branch-side extraction of the original labyrinth configuration for clean reuse.
3. A branch safety-margin function that computes distance to the nearest wall or obstacle, not just post-run validation metrics.
4. An obstacle-aware DeePC wrapper that can augment the original optimization with new inequalities.
5. A representation of obstacle or wall constraints over the prediction horizon.
6. A soft formulation using slack variables penalized in the cost.
7. A successive-linearization loop for nonconvex obstacle constraints.
8. A comparison layer between:
   - original baseline DeePC;
   - soft obstacle-aware DeePC;
   - iterative obstacle-aware DeePC.

## Key conclusion

The original labyrinth demo is already a solid base for the branch because it contains:

- a validated static labyrinth mission;
- a reusable DeePC stack with equality and inequality support;
- a clear plant surrogate;
- obstacle and wall geometry;
- strong post-run safety metrics.

What it does not contain yet is the core professor improvement:

- obstacle constraints inside the DeePC predictive optimization;
- slack-penalized obstacle handling;
- iterative or successive linearization of nonconvex safety constraints.
