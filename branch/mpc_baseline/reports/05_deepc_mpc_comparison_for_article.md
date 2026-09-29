# DeePC vs MPC Comparison for Article

## MPC baseline paragraph

For comparison with the data-driven branch controller, a model-based MPC baseline was implemented inside the `SwarmDrones/branch/mpc_baseline` folder. The implementation reuses the condensed quadratic-programming structure of the TCC source `mpc_controller_mosek_v2.m`, but adapts the prediction model to the same kinematic labyrinth plant used by the current DeePC branch. The resulting MPC operates on the four command channels `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]` and is solved online with the same receding-horizon path-following logic used in the labyrinth DeePC scenario.

## Comparison setup paragraph

The DeePC and MPC controllers were compared on the same SwarmDrones labyrinth environment, with identical obstacle geometry, identical reference-path generation logic, and the same initial condition. Three branch tests were considered: a baseline run without obstacle-aware correction, a soft obstacle-aware variant, and an iterative obstacle-aware variant based on successive linearization of cylindrical safety constraints. All metrics were computed from the realized closed-loop trajectories using the same safety-margin function `h(k)`.

## Tracking performance paragraph

Under the present branch configuration, the model-based MPC baseline produced lower tracking error than the current DeePC branch. In the baseline test, the DeePC controller achieved `RMSE_total = 0.2679 m`, whereas the MPC baseline achieved `RMSE_total = 0.2280 m`. A similar trend remained in the obstacle-aware runs: the DeePC soft and iterative variants remained near `0.2681 m`, while the corresponding MPC soft and iterative variants remained near `0.2271 m`. These results indicate that, on the current labyrinth surrogate plant, the adapted MPC achieves more accurate path tracking than the present DeePC branch tuning.

## Obstacle safety paragraph

The same branch comparison also showed that the current MPC variants reduced the severity of safety-margin violations relative to the present DeePC branch. The DeePC baseline reached `min_h = -0.1011 m` with `11` safety-violation samples, while the MPC baseline improved this to `min_h = -0.0456 m` with `3` violation samples. In the obstacle-aware case, the DeePC iterative variant still reached `min_h = -0.0985 m` with `11` violation samples, whereas the MPC iterative variant improved to `min_h = -0.0162 m` with `1` violation sample. The improvement is therefore substantial, although the current MPC branch is still not strictly collision-free in the most constrained segment.

## Control effort paragraph

The control-effort comparison must be interpreted carefully. In this branch, both controllers act on the native SwarmDrones command channels rather than physical motor angular velocities. Under that command-based measure, the MPC runs used higher total effort than the current DeePC branch: for example, the DeePC iterative run yielded `control_energy_total = 40.68`, whereas the MPC iterative run yielded `55.65`. Therefore, the present branch results suggest a tradeoff in which the model-based controller obtains better tracking and better safety margins at the cost of stronger command activity.

## Computational effort paragraph

The computational comparison in the present branch strongly favored the adapted MPC baseline. The DeePC baseline and obstacle-aware variants required mean solve times of roughly `0.058 s` to `0.128 s`, while the branch-side MPC runs remained near `0.0018 s` to `0.0025 s` per step. Since both methods remained feasible in all tested runs, the present branch results support the statement that the adapted condensed-QP MPC baseline is computationally lighter than the current branch DeePC implementation for this particular labyrinth scenario.

## Limitations paragraph

These conclusions must be framed as branch-scenario results, not as a universal DeePC-versus-MPC verdict. The present MPC baseline is not the original 12-state Crazyflie controller simulated on the IDSIA dataset; it is a branch-side adaptation of the TCC condensed-QP structure to the SwarmDrones labyrinth surrogate. Likewise, the branch DeePC controller uses the original labyrinth command interface rather than the motor-speed interface used in the IDSIA DeePC article draft. Therefore, the current comparison is suitable for reporting a transparent model-based baseline inside the labyrinth branch, but it should not yet be written as a definitive comparison between motor-speed DeePC and full-physics quadrotor MPC.
