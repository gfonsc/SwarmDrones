# Next Steps for Article

## 1. Suggested methodology text

Suggested wording:

> The original SwarmDrones labyrinth DeePC example was preserved as a read-only reference. A branch-only extension was then developed to incorporate obstacle-aware predictive constraints without modifying the original DeePC, database, or plant files. The branch reuses the original offline DeePC model and augments the online quadratic program with branch-side slack variables and linearized obstacle constraints built along the prediction horizon.

## 2. Suggested obstacle-avoidance formulation text

Suggested wording:

> Obstacle avoidance was introduced through local linearizations of the safety boundary around the nominal predicted trajectory. For each active obstacle feature, a local outward normal and a boundary point on the inflated unsafe set were computed, and the predictive output trajectory was constrained through linear half-space inequalities. To preserve feasibility, one nonnegative slack variable was associated with each active linearized constraint, and the sum of squared slack variables was penalized in the DeePC objective.

Suggested wording for the iterative version:

> Because the obstacle-avoidance geometry is nonconvex, a successive-linearization strategy was also implemented. At each control step, the nominal predicted trajectory was initialized from the previous prediction or the reference horizon, the obstacle constraints were linearized around this nominal path, the augmented DeePC QP was solved, and the nominal path was updated. This process was repeated for a small fixed number of iterations or until the prediction residual became sufficiently small.

## 3. Suggested results text

Suggested wording:

> The branch replay reproduced the original labyrinth DeePC example with full feasibility. The baseline branch controller obtained a position RMSE of 0.2679 m and a minimum safety margin of -0.1011 m relative to the inflated obstacle/wall safety set. The soft obstacle-aware extension and the iterative extension remained fully feasible and slightly improved the worst-case obstacle margin to approximately -0.0984 m, while keeping the tracking RMSE nearly unchanged. These results validate the branch implementation and indicate that the proposed constrained DeePC structure is operational, although further tuning is still required to obtain a clearly positive safety margin along the full trajectory.

## 4. Claims supported by the current branch results

- The original labyrinth DeePC example can be replayed entirely from the branch.
- The obstacle-aware DeePC wrapper can be implemented without modifying the original SwarmDrones files.
- A slack-enabled obstacle-aware DeePC QP can be solved reliably on the labyrinth example.
- A successive-linearization DeePC variant can be solved reliably on the same example.
- The obstacle-aware variants produce a small improvement in minimum obstacle clearance while keeping tracking performance nearly unchanged.

## 5. Claims not yet supported

- Full safety satisfaction with positive margin everywhere.
- A strong advantage of the iterative method over the soft method.
- A definitive DeePC-versus-MPC conclusion.
- A final obstacle-avoidance controller ready for publication without further tuning.

## 6. What should be sent to the advisor

Recommended package to send:

1. The branch folder path.
2. The audit reports:
   - `01_original_demo_audit.md`
   - `02_dependency_map.md`
   - `03_branch_implementation_plan.md`
   - `04_obstacle_deepc_formulation.md`
   - `05_results_summary.md`
3. The main scripts:
   - `run_01_original_demo_replay.m`
   - `run_02_branch_labyrinth_baseline.m`
   - `run_03_branch_soft_obstacle_deepc.m`
   - `run_04_branch_iterative_obstacle_deepc.m`
   - `run_05_branch_compare_variants.m`
4. The key result files:
   - `results/baseline_results.mat`
   - `results/soft_obstacle_results.mat`
   - `results/iterative_obstacle_results.mat`
   - `tables/branch_metrics.csv`
5. The paper-ready figures in `figures/paper_ready/`.

## 7. Recommended technical next step

The next technical step should be:

1. keep the current branch solver structure;
2. increase the predictive look-ahead beyond `T_f = 8`;
3. retune obstacle activation and nominal initialization around obstacle 4;
4. only then revisit wall constraints and MPC comparison.

## 8. Recommendation about article framing

Best current framing:

- present this branch as a working obstacle-aware DeePC prototype built from the original labyrinth example;
- claim successful implementation and feasibility;
- claim preliminary safety improvement;
- reserve stronger performance claims for the next tuning stage.
