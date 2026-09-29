clear; clc; close all;

paths = run_00_setup_final_comparison_paths();
common = final_load_common_reference();

deepcResult = final_run_deepc_tracking(common, 'A', 'UseObstacles', false);
mpcResult = final_run_mpc_tracking(common, 'A', 'UseObstacles', false);
motorValidation = final_validate_motor_efforts(deepcResult, mpcResult);

save(fullfile(paths.resultsDir, 'scenario_A_no_obstacle_deepc.mat'), 'deepcResult', 'common');
save(fullfile(paths.resultsDir, 'scenario_A_no_obstacle_mpc.mat'), 'mpcResult', 'common');

reportFile = fullfile(paths.reportsDir, '01_tracking_no_obstacle_validation.md');
fid = fopen(reportFile, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Scenario A - No-Obstacle Tracking Validation\n\n');
fprintf(fid, '- DeePC run successful: `%d`\n', all(deepcResult.log.success));
fprintf(fid, '- MPC run successful: `%d`\n', all(mpcResult.log.success));
fprintf(fid, '- Same reference used: `yes`\n');
fprintf(fid, '- Same initial condition used: `yes`\n');
fprintf(fid, '- Same number of samples: `%d`\n', numel(common.t_ref));
fprintf(fid, '- Sampling time: `%.3f s`\n', common.dt);
fprintf(fid, '- Time vectors aligned: `%d`\n\n', isequal(deepcResult.log.time, mpcResult.log.time));
fprintf(fid, '- Reference source: `%s`\n', common.referenceSourceA);
fprintf(fid, '- Labyrinth walls preserved for plotting/context: `yes`\n');
fprintf(fid, '- Cylindrical obstacles active: `no`\n\n');
fprintf(fid, '## Metrics\n\n');
fprintf(fid, '| Controller | RMSE_total (m) | Max error (m) | Final error (m) | Mean solve time (s) | Feasibility |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|\n');
fprintf(fid, '| DeePC | %.6f | %.6f | %.6f | %.6f | %.3f |\n', ...
    deepcResult.metrics.RMSE_total, deepcResult.metrics.max_error_total, ...
    deepcResult.metrics.final_error_total, deepcResult.metrics.mean_solver_time, ...
    deepcResult.metrics.feasibility_rate);
fprintf(fid, '| MPC | %.6f | %.6f | %.6f | %.6f | %.3f |\n\n', ...
    mpcResult.metrics.RMSE_total, mpcResult.metrics.max_error_total, ...
    mpcResult.metrics.final_error_total, mpcResult.metrics.mean_solver_time, ...
    mpcResult.metrics.feasibility_rate);

if deepcResult.metrics.RMSE_total < mpcResult.metrics.RMSE_total
    better = 'DeePC';
else
    better = 'MPC';
end
fprintf(fid, '## Conclusion\n\n');
fprintf(fid, 'Both controllers ran over the same nominal labyrinth setpoint reference and time window. Based on RMSE_total, `%s` tracks better in Scenario A.\n\n', better);
fprintf(fid, 'Motor-effort comparison type: `%s`. Valid for article as normalized equivalent effort: `%d`.\n', ...
    motorValidation.mappingType, motorValidation.validForArticle);

fprintf('\nScenario A complete.\n');
fprintf('  DeePC RMSE: %.4f m\n', deepcResult.metrics.RMSE_total);
fprintf('  MPC RMSE:   %.4f m\n', mpcResult.metrics.RMSE_total);
