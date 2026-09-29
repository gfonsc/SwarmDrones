# Signal Compatibility Report

## 1. Time and Reference Alignment
- **Same reference trajectory?** Spatially, yes. Both traverse the identical geometric labyrinth path. Temporally, no. 
- **Same time vector?** No. DeePC ran for 240 samples ($t_{end}=23.9$s), while MPC ran for 214 samples ($t_{end}=21.3$s).
- **Same duration?** No. MPC completed the trajectory faster. As a result, point-by-point time-domain subtraction (e.g., subtracting MPC $x(t)$ from DeePC $x(t)$) is mathematically invalid without resampling or spatial matching.

## 2. Trajectory Properties
- **Coordinate frame and units:** Both controllers output 3D position in the World Cartesian frame (meters), making spatial 2D and 3D plots perfectly valid for comparison.
- **Trajectory type:** Both `log.pos` arrays represent the actual executed closed-loop trajectory, not the open-loop prediction.

## 3. Input Signals Compatibility
- **Same input type?** No. DeePC explicitly uses four individual motor angular velocities. The MPC, based on standard analytical formulations, outputs virtual controls (e.g., collective thrust, roll rate, pitch rate, yaw rate) before allocation.
- **MPC Motor variables:** The 4 channels in the MPC `log.u` represent these virtual controls, not individual motors. 
- **MPC "Motor 4" anomaly:** The 4th channel (likely yaw rate command) sums to an extremely small value over the entire run because the reference yaw is constant, making it appear as "exactly zero" compared to motor speeds of thousands of rad/s.
- **Validity:** Direct overlay plotting of DeePC motors against MPC virtual inputs is conceptually invalid and must be removed or separated.

## 4. Euler Angles Validity
- **Availability:** DeePC does not track Euler angles internally. The MPC `log.euler` array exists but contains `NaN` values.
- **Validity:** The Euler angle plot is invalid because neither controller provides physically meaningful, loggable attitude states in this specific closed-loop implementation.

## 5. Conclusion on Invalid Plots
1. **Euler Angles Comparison:** Invalid. Data is absent or `NaN`.
2. **Motor Efforts Comparison:** Invalid. Comparing physical motor speeds against virtual control variables.
3. **Point-by-Point Tracking Error (over time):** Invalid due to differing time vectors (240 vs 214 samples).
4. **XYZ over Time:** Technically valid to plot side-by-side, but the visual misalignment is caused by the differing finish times, not necessarily poor spatial tracking.
