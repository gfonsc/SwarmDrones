# Recommended Article Figures

Based on the technical audit of the simulation signals, the following decisions guarantee the scientific validity of Section 6.

## 1. Usable Figures (Paper-Safe)
- **2D No-Obstacle Trajectory:** `usable_as_is` (Geometrically sound and correctly mapped).
- **2D Obstacle Trajectory:** `usable_as_is` (Correctly plots executed trajectories and geometric constraint regions).
- **3D Obstacle Trajectory:** `usable_as_is` (Spatially accurate representation of the evasive maneuvers).
- **State Machine Activation:** `usable_as_is` (Successfully logs when the successive linearization triggers).
- **Safety Margin h(k):** `usable_as_is` (Accurately plots the constraint evaluation, even with varying time lengths).

## 2. Usable with Caveats
- **X, Y, Z Tracking Comparison:** `usable_with_caveats` (The data is valid over independent time axes, but the visual divergence near $t=21s$ must be explicitly captioned as a consequence of MPC finishing the course faster, not necessarily tracking divergence).

## 3. Invalid Figures (Do Not Use)
- **Euler Angle Comparison:** `invalid_do_not_use` (Data is absent for DeePC and `NaN` for MPC. Do not fabricate).
- **Motor Effort Comparison:** `invalid_do_not_use` (Comparing physical motor angular velocities against virtual body/collective rates).

## 4. Required Action
Regenerate a clean folder containing only the usable figures. Do not attempt to synthesize Euler angles or normalize the non-equivalent motor efforts for the final manuscript.
