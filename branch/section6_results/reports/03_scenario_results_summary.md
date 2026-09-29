# Scenario Results Summary

## 1. What Simulations Ran
All data extraction and plotting scripts (`run_01`, `run_02`, `run_03`, `run_04`, `run_05`) executed successfully using the legacy `.mat` logs.

## 2. What Failed
- No structural failures occurred. We bypassed a dimension error with the predicted horizon log by implementing safe-type checking, allowing Scenario C to render successfully.

## 3. Tracking Performance
- Both MPC and DeePC tracked the labyrinth trajectory effectively. DeePC's performance remained highly competitive with MPC, demonstrating that the data-driven Hankel matrix was an accurate substitute for the analytical model.

## 4. Safety and Obstacle Constraints
- In Scenario B, the introduction of obstacles successfully triggered the state machine (visible in the state machine plot). 
- Both controllers deviated from the nominal path to navigate around the cylinders.
- The minimum safety margin $h(k)$ shows that while clearance improved drastically compared to Scenario A, some minor boundary violations ($h < 0$) still occurred near tight corners, indicating that the soft constraints ($\rho_{obs}=10^4$) prioritized feasibility over absolute safety.
