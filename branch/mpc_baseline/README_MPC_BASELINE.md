# MPC Baseline Branch Package

This folder contains the branch-only MPC baseline used to compare against the existing DeePC labyrinth branch.

Scope:

- Reuse the existing `SwarmDrones/branch` labyrinth scenario, reference path, safety-margin logic, and DeePC result files.
- Inspect and adapt the read-only source MPC project in `TCC/mpc-matlab`.
- Keep all new code, copied files, results, figures, tables, and reports inside `SwarmDrones/branch/mpc_baseline/`.

Important limitations:

- No original file outside `branch/` is modified.
- The branch MPC uses the same labyrinth plant surrogate as the DeePC branch for fair closed-loop comparison.
- The TCC quadrotor MPC source is still audited and reused as the template for the condensed QP formulation, weighting structure, and solver strategy.
- The labyrinth plant is kinematic, so roll and pitch are not physically simulated in the branch comparison. Yaw is available.

Execution order:

1. `run_00_setup_mpc_branch_paths`
2. `run_01_validate_mpc_source_replay`
3. `run_02_mpc_labyrinth_baseline`
4. `run_03_mpc_labyrinth_obstacle_aware`
5. `run_04_compare_deepc_vs_mpc_all_scenarios`
6. `run_05_generate_paper_figures`
