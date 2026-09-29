# Section 6 Simulation Plan

## 1. Scenarios
- **Scenario A:** Trajectory tracking without obstacles (plots baseline run, omitting obstacle rendering).
- **Scenario B:** Trajectory tracking with obstacles (plots iterative DeePC and MPC with obstacle geometry).
- **Scenario C:** Time-lagged obstacle visualization (plots a snapshot showing past executed paths, current positions, and predicted nominal horizon).

## 2. Controllers
- **DeePC:** The data-driven predictive controller.
- **MPC:** The model-based predictive controller (baseline comparison).

## 3. Expected Plots
For each scenario:
- 2D/3D Trajectory Maps
- XYZ and Euler Angle Tracking
- Motor Efforts
- State Machine Activation
- Safety Margin $h(k)$

## 4. Expected Metrics
- RMSE and MAE in X, Y, Z.
- Minimum distance to obstacle.
- Minimum safety margin and violation depth.
- Control energy and variation.
