# Available Data Audit

## 1. Existing DeePC Results
- `baseline_results.mat`: Unconstrained tracking in the labyrinth.
- `soft_obstacle_results.mat`: Soft obstacle-aware DeePC tracking.
- `iterative_obstacle_results.mat`: Successive linearization obstacle-aware DeePC tracking.
- `original_demo_replay_results.mat`: Baseline without branch wrappers.
- `comparison_results.mat`: Holds aggregated comparison structures.

## 2. Existing MPC Results
- `mpc_baseline_results.mat`: Unconstrained tracking in the labyrinth.
- `mpc_obstacle_results.mat`: Contains both soft and iterative obstacle-aware MPC tracking.
- `deepc_mpc_comparison_results.mat`: Aggregated comparison.

## 3. Available Scenarios
The available simulations were all executed on the SwarmDrones Labyrinth scenario.
This natively represents a "with obstacles" scenario (Scenario B).
- **Scenario B (With Obstacles):** Already fully available through `iterative_obstacle_results.mat` and `mpc_obstacle_results.mat`.
- **Scenario A (No Obstacle):** Currently, the "baseline" runs evaluate tracking inside the labyrinth *ignoring* the obstacles (but the geometric metrics are still calculated against the obstacles). For a pure "No Obstacle Tracking" plot, we can just plot the baseline results without rendering the obstacles.
- **Scenario C (Time-Lagged Obstacles):** No data explicitly simulates time-lagged or delayed detection currently. This scenario needs either a dedicated simulation script or a visualization trick using the existing iterations/predictions. The easiest and most aligned approach is to visualize the *predicted* nominal boundaries at time $k$ alongside the *actual* executed path.

## 4. Fields in Result Files
The result logs contain:
- Time vector: `time`
- Reference trajectory: `ref`
- Positions: `pos`
- Roll, pitch, yaw: `euler` (Only in MPC. DeePC does not extract Euler angles directly since $y=p$, so it'll be missing or empty for DeePC).
- Motor inputs: `u`
- Safety margin: `safetyMargin` array (contains `margin`, `obsClearance`, `wallClearance`).
- Obstacles: Available in `result.scenario.obstacles`.
- State-machine states: Not explicitly recorded as a categorical array, but we have `iterationCount` and `constraintCount` which implicitly track the "Constraint Generation / Iterative Update" state activation.
- Solver time: `solveTime`
- Feasibility flags: `success`

## 5. Missing Data
- Euler angles for DeePC (expected, since DeePC maps inputs directly to 3D positions without explicitly estimating internal angles).
- Explicit categorical state-machine log (we will reconstruct it logically from `constraintCount > 0`).

## 6. What Needs to be Rerun
No simulations strictly need to be rerun from scratch because the existing data inside `results/` already captures the full closed-loop flights. However, we will create clean scripts (`run_01`, `run_02`, `run_03`) that simply load these robust `.mat` files and construct the standardized structures required for Section 6.

## 7. Reusability of Figures
Existing figures in `figures/paper_ready/` were exported but have dark/black backgrounds. We must regenerate them using a strict white-background `apply_plot_style` protocol to meet academic standards.
