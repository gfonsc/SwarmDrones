clear; clc; close all;

paths = run_00_setup_final_comparison_paths();
common = final_load_common_reference();

deepcObstacleOptions = struct();
deepcObstacleOptions.RhoObs = 1e5;
deepcObstacleOptions.SafeDistance = common.scenarioB.safeDistance;
deepcObstacleOptions.ActivationDistance = 1.00;
deepcObstacleOptions.MaxConstraintsPerStep = 2;
deepcObstacleOptions.IncludeWalls = false;
deepcObstacleOptions.IncludeObstacles = true;
deepcObstacleOptions.MaxIter = 4;
deepcObstacleOptions.Tolerance = 1e-3;

mpcObstacleOptions = struct();
mpcObstacleOptions.RhoSlack = 1e4;
mpcObstacleOptions.SafeDistance = common.scenarioB.safeDistance;
mpcObstacleOptions.ActivationDistance = 0.60;
mpcObstacleOptions.MaxConstraintsPerStep = 2;
mpcObstacleOptions.IncludeWalls = false;
mpcObstacleOptions.IncludeObstacles = true;
mpcObstacleOptions.MaxIter = 4;
mpcObstacleOptions.Tolerance = 1e-3;

deepcResult = final_run_deepc_tracking(common, 'B', ...
    'UseObstacles', true, 'ObstacleOptions', deepcObstacleOptions);
mpcResult = final_run_mpc_tracking(common, 'B', ...
    'UseObstacles', true, 'ObstacleOptions', mpcObstacleOptions);
motorValidation = final_validate_motor_efforts(deepcResult, mpcResult);

save(fullfile(paths.resultsDir, 'scenario_B_obstacle_deepc.mat'), 'deepcResult', 'common');
save(fullfile(paths.resultsDir, 'scenario_B_obstacle_mpc.mat'), 'mpcResult', 'common');

reportFile = fullfile(paths.reportsDir, '02_obstacle_scenario_validation.md');
fid = fopen(reportFile, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Scenario B - Obstacle Tracking Validation\n\n');
fprintf(fid, '- Obstacles active: `yes`\n');
fprintf(fid, '- Number of obstacles: `%d`\n', size(common.scenarioB.obstacles, 1));
fprintf(fid, '- Same obstacles used by DeePC and MPC: `yes`\n');
fprintf(fid, '- Same reference used: `yes`\n');
fprintf(fid, '- Reference source: `%s`\n', common.referenceSourceB);
fprintf(fid, '- Same initial condition used: `yes`\n');
fprintf(fid, '- Time vectors aligned: `%d`\n', isequal(deepcResult.log.time, mpcResult.log.time));
fprintf(fid, '- Labyrinth walls preserved for plotting/context: `yes`\n');
fprintf(fid, '- Safety margin uses cylindrical obstacles only: `yes`\n');
fprintf(fid, '- Wall constraints included in final safety metric: `no`\n\n');
fprintf(fid, '## Metrics\n\n');
fprintf(fid, '| Controller | RMSE_total (m) | min h (m) | Violations | Mean solve time (s) | Feasibility |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|\n');
fprintf(fid, '| DeePC | %.6f | %.6f | %.0f | %.6f | %.3f |\n', ...
    deepcResult.metrics.RMSE_total, deepcResult.metrics.min_h, ...
    deepcResult.metrics.num_safety_violations, deepcResult.metrics.mean_solver_time, ...
    deepcResult.metrics.feasibility_rate);
fprintf(fid, '| MPC | %.6f | %.6f | %.0f | %.6f | %.3f |\n\n', ...
    mpcResult.metrics.RMSE_total, mpcResult.metrics.min_h, ...
    mpcResult.metrics.num_safety_violations, mpcResult.metrics.mean_solver_time, ...
    mpcResult.metrics.feasibility_rate);

if deepcResult.metrics.min_h >= 0
    deepcSafe = 'yes';
else
    deepcSafe = 'no';
end
if mpcResult.metrics.min_h >= 0
    mpcSafe = 'yes';
else
    mpcSafe = 'no';
end

fprintf(fid, '## Conclusion\n\n');
fprintf(fid, '- DeePC satisfies `h(k) >= 0`: `%s`\n', deepcSafe);
fprintf(fid, '- MPC satisfies `h(k) >= 0`: `%s`\n', mpcSafe);
if deepcResult.metrics.min_h > mpcResult.metrics.min_h
    safer = 'DeePC';
else
    safer = 'MPC';
end
fprintf(fid, '- Better safety margin in Scenario B: `%s`\n', safer);
fprintf(fid, '- Obstacle avoidance is inside both optimization problems as soft linearized constraints with slack.\n');
fprintf(fid, '- Motor-effort comparison type: `%s`; valid as normalized equivalent effort: `%d`.\n', ...
    motorValidation.mappingType, motorValidation.validForArticle);

fprintf('\nScenario B complete.\n');
fprintf('  DeePC RMSE: %.4f m, min h: %.4f m\n', deepcResult.metrics.RMSE_total, deepcResult.metrics.min_h);
fprintf('  MPC RMSE:   %.4f m, min h: %.4f m\n', mpcResult.metrics.RMSE_total, mpcResult.metrics.min_h);
