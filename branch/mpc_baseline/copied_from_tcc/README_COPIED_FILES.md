# Copied TCC MPC Files

This folder contains read-only source copies from:

`C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\TCC\mpc-matlab`

## File registry

1. Original: `TCC/mpc-matlab/crazyflie_params.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/crazyflie_params.m`
   Why: source physical parameter reference
   Modified: no

2. Original: `TCC/mpc-matlab/linearize_quadrotor.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/linearize_quadrotor.m`
   Why: source linear model reference
   Modified: no

3. Original: `TCC/mpc-matlab/quadrotor_dynamics.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/quadrotor_dynamics.m`
   Why: source nonlinear model reference
   Modified: no

4. Original: `TCC/mpc-matlab/design_mpc_quadrotor.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/design_mpc_quadrotor.m`
   Why: source MPC Toolbox design reference
   Modified: no

5. Original: `TCC/mpc-matlab/mpc_controller_mosek.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/mpc_controller_mosek.m`
   Why: source condensed QP reference
   Modified: no

6. Original: `TCC/mpc-matlab/mpc_controller_mosek_v2.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/mpc_controller_mosek_v2.m`
   Why: main structural template for the branch condensed-QP controller
   Modified: no

7. Original: `TCC/mpc-matlab/test_mpc_quadrotor.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/test_mpc_quadrotor.m`
   Why: source replay reference
   Modified: no

8. Original: `TCC/mpc-matlab/test_mpc_mosek_v2.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/test_mpc_mosek_v2.m`
   Why: source replay reference
   Modified: no

9. Original: `TCC/mpc-matlab/mpc_top10_evaluation.m`
   Copied: `branch/mpc_baseline/copied_from_tcc/mpc_top10_evaluation.m`
   Why: source tuning summary reference
   Modified: no

## Branch adaptation note

The branch implementation does not edit these copies directly at this stage. Instead, new wrapper files inside `branch/mpc_baseline/functions/` adapt the TCC condensed-QP approach to the existing SwarmDrones labyrinth plant and obstacle geometry.
