# Final Result Validity Audit

## 1. Overall Conclusions
- **Are the current DeePC results valid?** `usable_with_caveats`
- **Are the current MPC results valid?** `usable_with_caveats`
- **Are DeePC and MPC directly comparable?** Spatially: YES. Temporally: NO. Their control inputs: NO.
- **Does MPC truly improve trajectory tracking?** Neither showed absolute superiority. The data proves DeePC provides competitive, model-free tracking identical to the analytical model constraint.
- **Does MPC truly improve obstacle safety?** No. Both methods use the same successive linearization logic, both yielding the exact same minor safety violations on tight corners.

## 2. Status of Specific Plots
1. **2D and 3D Trajectory Maps:** `ready_for_article`. These correctly overlay the spatial paths without time-domain alignment issues.
2. **XYZ Tracking over Time:** `usable_with_caveats`. Useful for demonstrating system response speed, but it must be clearly captioned that the lines separate near the end because MPC finishes the reference sequence faster (21.3s vs 23.9s), not because DeePC diverged spatially.
3. **Euler Angles:** `invalid_needs_fixing` (Discard entirely). Neither implementation logged true Euler angles.
4. **Motor Efforts:** `invalid_needs_fixing` (Discard entirely). Comparing physical angular velocities against virtual collective rates is scientifically meaningless.
5. **Safety Margin h(k):** `ready_for_article`. Effectively proves the obstacle generation algorithm works, explicitly showing the soft margin constraint behavior.

## 3. What Can Be Claimed in the Paper
- **Claim:** The data-driven predictive controller successfully tracked a highly dynamic trajectory in a confined labyrinth environment.
- **Claim:** DeePC achieved spatial tracking comparable to a state-of-the-art analytical MPC, without requiring a parametric dynamic model.
- **Claim:** Successive linearization successfully deflected the quadrotor from collision courses, evaluating soft constraints over the prediction horizon.

## 4. What MUST NOT Be Claimed
- **Do not claim perfect, absolute collision avoidance.** The minimum $h(k)$ fell to $-0.0985$m. It prevented a physical crash, but the mathematical safety buffer was temporarily violated.
- **Do not claim control effort superiority.** The input vectors log.u have different fundamental physical meanings in these scripts and cannot be compared.
- **Do not claim internal attitude stability using Euler plots.** The closed-loop DeePC maps directly to position and velocity; intermediate attitudes are unobservable in the existing logged dataset.
