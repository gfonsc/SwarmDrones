# Limitations and Next Steps

## Main assumptions made in the branch adaptation

1. The TCC condensed-QP MPC structure was reused conceptually, but the prediction model was adapted to the SwarmDrones labyrinth plant rather than the original 12-state quadrotor model.
2. The branch comparison uses the same command-based plant for both DeePC and MPC:
   - `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`
3. Obstacle avoidance was implemented through linearized cylindrical inequalities with slack rather than exact nonlinear constraints.
4. The present obstacle-aware MPC uses the labyrinth obstacle cylinders directly and monitors safety through the same branch safety-margin function already used by the DeePC branch.

## Input-representation limitation

The current branch comparison does not use the motor-speed input representation from the IDSIA DeePC article draft. It uses the native SwarmDrones labyrinth command interface for both methods. This is appropriate for a fair branch comparison, but it is not equivalent to the motor-speed DeePC formulation reported in the article draft.

## Obstacle-equivalence limitation

The obstacle-aware MPC and the obstacle-aware DeePC branch do not yet represent the exact same optimization structure:

- DeePC uses the original branch obstacle wrapper built around the Hankel-prediction pipeline.
- MPC uses an adapted linear model with linearized constraints in the condensed-QP space.

Both methods are evaluated with the same geometry and the same `h(k)` definition, so the comparison is consistent at the scenario level. However, the internal model assumptions differ.

## Attitude limitation

The branch labyrinth plant does not provide full roll and pitch dynamics for either method. Yaw is available, but roll and pitch are unavailable and are therefore plotted as unavailable rather than estimated or fabricated.

## Safety limitation

Neither the present DeePC branch nor the present obstacle-aware MPC baseline fully achieved `h(k) >= 0` for the entire labyrinth traversal. The MPC branch reduced the violation depth significantly, but residual negative safety-margin samples remain. The current results therefore support improvement claims, not strict safety-guarantee claims.

## Suggested next steps

1. Add wall constraints explicitly into the obstacle-aware MPC and retune the activation logic near the last serpentine segment.
2. Increase or retune the horizon for the obstacle-aware variants and repeat the comparison.
3. Revisit the DeePC obstacle-constraint activation and nominal initialization so that the current branch DeePC can reduce its remaining `h(k) < 0` interval.
4. If article scope allows, add a higher-fidelity model-based baseline that uses the copied 12-state TCC quadrotor dynamics on a scenario aligned with the labyrinth geometry.
5. If a stronger paper claim is required, the next milestone should be a scenario where both methods remain fully feasible and at least one method achieves strictly nonnegative safety margin throughout the critical obstacle segment.

## Are the current results sufficient for article submission?

They are sufficient to document:

- that a transparent model-based MPC baseline was implemented in the branch;
- that the same labyrinth scenario and metrics were used;
- that the current branch-side MPC outperformed the present DeePC branch on tracking and safety metrics in the tested setup.

They are not sufficient to support:

- a claim of complete collision-free obstacle avoidance;
- a claim of definitive superiority of DeePC over MPC;
- a claim that the comparison already reflects the final physical quadrotor control architecture.
