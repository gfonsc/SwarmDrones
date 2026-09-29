# Plotting Issues Report

## 1. Time and Reference Alignment (Task 5)
- **Time Misalignment:** The DeePC and MPC simulations did not log identical time sequences. DeePC ran for 240 samples ($t=23.9$s) while MPC ran for 214 samples ($t=21.3$s).
- **Impact:** Any point-by-point tracking error computation (e.g., $e(k) = x_{deepc}(k) - x_{mpc}(k)$) is completely invalid. Time-series overlays of XYZ coordinates are technically correct relative to their own time axes, but the lines will artificially diverge near the end because MPC finishes earlier. 
- **Recommendation:** Spatial 2D and 3D trajectory plots remain valid and are the most honest way to compare tracking performance.

## 2. Euler Angle Validity Check (Task 6)
- **Data Availability:** DeePC does not extract Euler angles, mapping motor inputs directly to 3D Cartesian space. MPC `log.euler` is present but filled with `NaN` values.
- **Impact:** The previous Euler angle plots were blank or invalid.
- **Recommendation:** Do not generate or include Euler angle plots in the final article.
- **Statement for Article:** "Euler-angle comparison is not reported because the implemented data-driven formulation natively bypasses internal attitude estimation, mapping inputs directly to Cartesian coordinates."

## 3. Motor Effort Validity Check (Task 7)
- **Data Types:** DeePC explicitly outputs four individual physical motor angular velocities (e.g., $2000-4000$ rad/s). 
- **MPC Control Outputs:** The 4-channel `u` array for MPC represents virtual control inputs (typically collective thrust and roll/pitch/yaw moments or rates), not physical motor speeds. 
- **Zero Anomaly:** The 4th channel of MPC `u` evaluates to nearly zero over the entire trajectory because it represents a yaw rate or differential moment command, which is identically zero for constant yaw tracking.
- **Impact:** The `section6_plot_motor_efforts.m` graph is comparing apples (physical motor speeds) to oranges (virtual body rates/thrust).
- **Recommendation:** This plot is scientifically invalid for a direct side-by-side comparison and must be discarded from the article.

## 4. Summary of Diagnostics
The raw data is sound, but it represents two fundamentally different mathematical formulations operating over slightly different completion times. The most scientifically rigorous visualizations are the spatial geometric plots (2D/3D maps) and independent performance metrics.
