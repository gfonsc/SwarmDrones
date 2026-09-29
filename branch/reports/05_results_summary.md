# Results Summary

## 1. Did the original labyrinth demo run?

Yes.

The branch-side replay reproduced the original labyrinth DeePC behavior without modifying the original project and saved the outputs only inside `branch/`.

## 2. Did the branch baseline reproduce the original behavior?

Yes.

The branch baseline matched the replay metrics because it reuses the same:

- labyrinth scenario;
- training database;
- DeePC hyperparameters;
- plant rollout;
- reference construction logic.

Baseline metrics:

- QP success rate: `1.0000`
- position RMSE: `0.267873 m`
- final error: `0.273569 m`
- min obstacle clearance: `0.078934 m`
- min wall clearance: `0.317616 m`
- min safety margin: `-0.101066 m`

## 3. Did the safety-margin computation work?

Yes.

The branch computes:

- obstacle clearance;
- wall clearance;
- combined safety margin `h(k)`.

It correctly identified the worst-case margin near obstacle 4 in the last serpentine segment.

## 4. Did the soft obstacle version improve safety?

Slightly.

Soft obstacle-aware DeePC metrics:

- position RMSE: `0.268084 m`
- final error: `0.275038 m`
- min obstacle clearance: `0.081637 m`
- min safety margin: `-0.098363 m`
- mean slack: `3.03e-05`

Compared with the baseline, the soft version improved:

- min obstacle clearance by about `2.70 mm`;
- min safety margin by about `2.70 mm`.

So the improvement is real but modest.

## 5. Did the iterative version improve safety or reduce violations?

Not materially in the saved default branch configuration.

Iterative obstacle-aware DeePC metrics:

- position RMSE: `0.268105 m`
- final error: `0.274840 m`
- min obstacle clearance: `0.081511 m`
- min safety margin: `-0.098489 m`
- mean slack: `3.00e-05`
- mean iterations: `2.00`

It stayed fully feasible and converged in a small number of iterations, but it did not clearly outperform the soft single-pass version.

## 6. What tradeoff appeared between tracking and safety?

The current tradeoff is mild:

- tracking RMSE stayed almost unchanged across all three variants;
- obstacle clearance improved slightly with the obstacle-aware variants;
- the improvement was not strong enough to eliminate all safety-margin violations.

This means the current branch extension is a good methodological prototype, but not yet a strong performance result.

## 7. What failed?

Nothing failed structurally:

- all branch runs remained feasible;
- no original project file was modified;
- the branch-side replay, baseline, soft, iterative, and comparison scripts all ran.

What did not yet succeed is the stronger scientific goal:

- the obstacle-aware controller does not yet produce a clearly safer final path with positive `h(k)` everywhere.

## 8. What still needs manual correction or tuning?

The branch still needs tuning in at least one of these directions:

- longer prediction horizon than `T_f = 8`;
- stronger or more targeted obstacle activation around obstacle 4;
- better nominal initialization for the iterative loop;
- better treatment of wall constraints so they do not conflict with obstacle avoidance;
- scenario-specific weighting for the last serpentine obstacle passage.

## 9. Which figures are ready for the article?

The branch generated paper-ready figures in:

- `branch/figures/paper_ready/labyrinth_baseline_vs_obstacle_deepc.png`
- `branch/figures/paper_ready/safety_margin_comparison.png`
- `branch/figures/paper_ready/iterative_convergence.png`
- `branch/figures/paper_ready/control_inputs_comparison.png`

These are suitable for:

- methodology illustrations;
- qualitative comparison figures;
- reporting that the branch prototype exists and runs.

## 10. Which claims are safe to write in the article?

Safe claims:

- a branch-side obstacle-aware DeePC extension was implemented without modifying the original project;
- the extension reuses the original DeePC database, solver backend, and labyrinth scenario;
- both a soft obstacle-aware formulation and an iterative successive-linearization formulation run successfully;
- all tested variants remained feasible on the original labyrinth scenario;
- the obstacle-aware variants produced a small improvement in the worst-case obstacle clearance.

Not yet safe claims:

- the current branch guarantees safe labyrinth traversal with positive safety margin everywhere;
- the iterative method is clearly better than the soft method;
- the current DeePC branch is already ready as a final obstacle-avoidance benchmark result.
