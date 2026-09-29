# SwarmDrones Branch

This folder contains the branch-only adaptation of the original labyrinth DeePC example.

Rules followed in this branch:

- no original file in `SwarmDrones` was modified;
- all new scripts, functions, figures, reports, tables, and MAT results live inside `branch/`;
- the original labyrinth demo was first replayed faithfully from the branch;
- the obstacle-aware extension was added as a branch-side wrapper around the original DeePC stack.

Main entry scripts:

- `scripts/run_01_original_demo_replay.m`
- `scripts/run_02_branch_labyrinth_baseline.m`
- `scripts/run_03_branch_soft_obstacle_deepc.m`
- `scripts/run_04_branch_iterative_obstacle_deepc.m`
- `scripts/run_05_branch_compare_variants.m`

Main branch functions:

- `functions/branch_extract_demo_config.m`
- `functions/branch_run_labyrinth_rollout.m`
- `functions/branch_compute_safety_margin.m`
- `functions/branch_build_obstacle_constraints.m`
- `functions/branch_linearize_obstacle_constraints.m`
- `functions/branch_solve_obstacle_deepc.m`
- `functions/branch_plot_labyrinth_results.m`

Key outputs:

- `results/baseline_results.mat`
- `results/soft_obstacle_results.mat`
- `results/iterative_obstacle_results.mat`
- `results/comparison_results.mat`
- `tables/branch_metrics.csv`
- `figures/paper_ready/`
- `reports/`

Current status:

- the original branch replay works;
- the branch baseline works;
- the soft obstacle-aware DeePC wrapper works;
- the iterative obstacle-aware DeePC wrapper works;
- all variants remain feasible in the current labyrinth setup;
- the current obstacle-aware gains are modest, so the branch is a valid prototype and article base, but not yet a final safety-optimized controller.
