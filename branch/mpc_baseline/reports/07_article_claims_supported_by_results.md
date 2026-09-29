# Article Claims Supported by Results

## Supported claims

- A model-based MPC baseline was implemented entirely inside `SwarmDrones/branch/mpc_baseline` without modifying the original source projects.
- The MPC baseline was evaluated on the same SwarmDrones labyrinth scenario, obstacle geometry, reference-path logic, and initial condition used by the current DeePC branch.
- The branch-side MPC reused the condensed-QP design pattern of the TCC source and solved the resulting optimization online with MOSEK through the shared `solveQP` interface.
- Both the baseline and obstacle-aware MPC variants ran successfully with feasibility rate `1.0`.
- In the tested branch scenario, the adapted MPC baseline achieved lower closed-loop tracking RMSE than the current DeePC branch.
- In the tested branch scenario, the obstacle-aware MPC variants reduced the depth and count of safety-margin violations relative to the current DeePC branch.
- The comparison produced consistent `.mat`, `.csv`, `.fig`, and `.png` artifacts suitable for inclusion in the article workflow.

## Unsupported claims

- The proposed DeePC branch outperforms MPC in the current labyrinth scenario.
- The current obstacle-aware DeePC branch is already collision-free.
- The current obstacle-aware MPC branch is collision-free.
- The present branch comparison reproduces the exact motor-speed DeePC setting from the IDSIA article draft.
- The present branch comparison is a full-physics comparison between the original TCC quadrotor MPC and the IDSIA motor-speed DeePC controller.

## Conditional claims

- The adapted MPC baseline showed better tracking and smaller negative safety excursions than the current DeePC branch under the tested labyrinth surrogate scenario, although neither method fully eliminated negative safety-margin samples.
- The obstacle-aware formulations are practical and computationally tractable in the current branch implementation, but the safety results should be presented as preliminary until `h(k) >= 0` is achieved throughout the critical segment.
- The present DeePC-vs-MPC comparison is appropriate for the SwarmDrones branch article figures, provided the manuscript explicitly states that both methods are compared on the command-based labyrinth surrogate rather than the motor-speed IDSIA plant.
- The current figures are suitable for methodology and preliminary results sections, but strong claims about final obstacle-avoidance performance should remain qualified.
