clear; clc; close all;

paths = run_00_setup_final_comparison_paths();

Adeepc = load(fullfile(paths.resultsDir, 'scenario_A_no_obstacle_deepc.mat'), 'deepcResult', 'common');
Ampc = load(fullfile(paths.resultsDir, 'scenario_A_no_obstacle_mpc.mat'), 'mpcResult');
Bdeepc = load(fullfile(paths.resultsDir, 'scenario_B_obstacle_deepc.mat'), 'deepcResult', 'common');
Bmpc = load(fullfile(paths.resultsDir, 'scenario_B_obstacle_mpc.mat'), 'mpcResult');

scenarioA_deepc = Adeepc.deepcResult;
scenarioA_mpc = Ampc.mpcResult;
scenarioB_deepc = Bdeepc.deepcResult;
scenarioB_mpc = Bmpc.mpcResult;
common = Adeepc.common;

scenarioA_deepc.metrics = final_compute_metrics(scenarioA_deepc);
scenarioA_mpc.metrics = final_compute_metrics(scenarioA_mpc);
scenarioB_deepc.metrics = final_compute_metrics(scenarioB_deepc);
scenarioB_mpc.metrics = final_compute_metrics(scenarioB_mpc);

fullTable = [
    rowForResult('Scenario A - no obstacle', scenarioA_deepc)
    rowForResult('Scenario A - no obstacle', scenarioA_mpc)
    rowForResult('Scenario B - obstacle', scenarioB_deepc)
    rowForResult('Scenario B - obstacle', scenarioB_mpc)
];

writetable(fullTable, fullfile(paths.tablesDir, 'final_metrics_full.csv'));

paperTable = fullTable(:, {'scenario', 'controller', 'RMSE_total', 'max_error_total', ...
    'control_energy_total', 'min_h', 'num_safety_violations', 'mean_solver_time', ...
    'feasibility_rate', 'notes'});
writetable(paperTable, fullfile(paths.tablesDir, 'final_metrics_paper_table.csv'));

comparison = struct();
comparison.common = common;
comparison.scenarioA.deepc = scenarioA_deepc;
comparison.scenarioA.mpc = scenarioA_mpc;
comparison.scenarioB.deepc = scenarioB_deepc;
comparison.scenarioB.mpc = scenarioB_mpc;
comparison.fullTable = fullTable;
comparison.paperTable = paperTable;
save(fullfile(paths.resultsDir, 'final_comparison_results.mat'), 'comparison');

writeFairComparisonReport(paths, comparison);
writeArticleReadySummary(paths, comparison);
writeMotorValidityReport(paths, scenarioA_deepc, scenarioA_mpc);

fprintf('\nFinal metrics complete.\n');
disp(paperTable);

function T = rowForResult(scenarioName, result)
    m = result.metrics;
    T = table();
    T.scenario = string(scenarioName);
    T.controller = string(result.controller);
    T.RMSE_x = m.RMSE_x;
    T.RMSE_y = m.RMSE_y;
    T.RMSE_z = m.RMSE_z;
    T.RMSE_total = m.RMSE_total;
    T.MAE_x = m.MAE_x;
    T.MAE_y = m.MAE_y;
    T.MAE_z = m.MAE_z;
    T.max_error_total = m.max_error_total;
    T.final_error_total = m.final_error_total;
    T.control_energy_total = m.control_energy_total;
    T.control_energy_motor_1 = m.control_energy_motor_1;
    T.control_energy_motor_2 = m.control_energy_motor_2;
    T.control_energy_motor_3 = m.control_energy_motor_3;
    T.control_energy_motor_4 = m.control_energy_motor_4;
    T.control_variation_total = m.control_variation_total;
    T.max_control_effort = m.max_control_effort;
    T.min_distance_to_obstacle = m.min_distance_to_obstacle;
    T.min_h = m.min_h;
    T.num_safety_violations = m.num_safety_violations;
    T.max_violation_depth = m.max_violation_depth;
    T.mean_solver_time = m.mean_solver_time;
    T.max_solver_time = m.max_solver_time;
    T.feasibility_rate = m.feasibility_rate;
    if result.usesObstacles
        if contains(string(scenarioName), "Scenario B")
            T.notes = string('Original labyrinth training reference; labyrinth walls plotted; cylindrical obstacles active with soft linearized constraints.');
        else
            T.notes = string('Obstacle-aware run with soft linearized constraints.');
        end
    else
        T.notes = string('Nominal setpoint reference; labyrinth walls plotted; cylindrical obstacles inactive.');
    end
end

function writeFairComparisonReport(paths, comparison)
    A = comparison.scenarioA;
    B = comparison.scenarioB;
    fid = fopen(fullfile(paths.reportsDir, '03_deepc_mpc_fair_comparison_report.md'), 'w');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# DeePC vs MPC Fair Comparison Report\n\n');
    fprintf(fid, '## Scenarios\n\n');
    fprintf(fid, '- Scenario A: nominal labyrinth setpoint tracking with no cylindrical obstacles active.\n');
    fprintf(fid, '- Scenario B: original labyrinth training reference with the branch cylindrical obstacles active.\n\n');
    fprintf(fid, 'Within each scenario, DeePC and MPC use the same reference, same initial condition, same sampling time, and same fixed time window.\n');
    fprintf(fid, 'The labyrinth walls are preserved and drawn in both final trajectory figures.\n');
    fprintf(fid, 'Motor efforts are equivalent normalized command efforts, not physical motor speeds.\n\n');
    fprintf(fid, '## Main Results\n\n');
    fprintf(fid, '| Scenario | Controller | RMSE_total | min h | Control energy | Mean solve time |\n');
    fprintf(fid, '|---|---|---:|---:|---:|---:|\n');
    printRow(fid, 'A', A.deepc);
    printRow(fid, 'A', A.mpc);
    printRow(fid, 'B', B.deepc);
    printRow(fid, 'B', B.mpc);
    fprintf(fid, '\n');
    fprintf(fid, '- Better Scenario A tracking by RMSE: `%s`\n', betterRmse(A.deepc, A.mpc));
    fprintf(fid, '- Better Scenario B tracking by RMSE: `%s`\n', betterRmse(B.deepc, B.mpc));
    fprintf(fid, '- Better Scenario B safety margin: `%s`\n', betterSafety(B.deepc, B.mpc));
    fprintf(fid, '- Lower Scenario B equivalent control energy: `%s`\n', betterEnergy(B.deepc, B.mpc));
    fprintf(fid, '- Computationally heavier in Scenario B: `%s`\n\n', heavierSolver(B.deepc, B.mpc));
    fprintf(fid, '## Article Claims\n\n');
    fprintf(fid, 'Safe to write: within each scenario, DeePC and MPC use identical references, time bases, initial conditions, and obstacle definitions.\n');
    fprintf(fid, 'Safe to write: the labyrinth geometry is drawn in both Scenario A and Scenario B figures.\n');
    fprintf(fid, 'Safe to write: motor effort is reported as equivalent normalized command effort, not physical rotor speed.\n');
    fprintf(fid, 'Do not claim strict obstacle avoidance if either Scenario B `min_h` is negative.\n');
    fprintf(fid, 'Do not claim physical motor-speed comparison for this branch implementation.\n');
end

function writeArticleReadySummary(paths, comparison)
    A = comparison.scenarioA;
    B = comparison.scenarioB;
    fid = fopen(fullfile(paths.reportsDir, '05_article_ready_results_summary.md'), 'w');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# Article-Ready Results Summary\n\n');
    fprintf(fid, 'Scenario A evaluated trajectory tracking through the nominal labyrinth setpoints, with labyrinth walls drawn and no cylindrical obstacles active. DeePC obtained `RMSE_total = %.4f m`, while MPC obtained `RMSE_total = %.4f m` over the same time interval.\n\n', A.deepc.metrics.RMSE_total, A.mpc.metrics.RMSE_total);
    fprintf(fid, 'Scenario B used the original labyrinth training reference for both controllers, with the branch cylindrical obstacles active. DeePC obtained `RMSE_total = %.4f m` and `min_h = %.4f m`; MPC obtained `RMSE_total = %.4f m` and `min_h = %.4f m`.\n\n', B.deepc.metrics.RMSE_total, B.deepc.metrics.min_h, B.mpc.metrics.RMSE_total, B.mpc.metrics.min_h);
    fprintf(fid, 'Tracking should be interpreted using `RMSE_total`, maximum error, and final error from `final_metrics_full.csv`, because both controllers were evaluated on executed trajectories rather than predictions.\n\n');
    fprintf(fid, 'Safety should be described carefully. A positive `min_h` would indicate clearance from the active cylindrical obstacles; a negative value means residual safety-margin violation remains and should not be described as successful obstacle avoidance.\n\n');
    fprintf(fid, 'Control effort is reported as equivalent normalized command effort. This supports a relative smoothness and effort comparison inside the branch simulation, but it is not a physical rotor-speed measurement.\n\n');
    fprintf(fid, 'The main limitation is that the final comparison uses the SwarmDrones branch kinematic command interface, not a full physical motor-speed quadrotor model.\n');
end

function writeMotorValidityReport(paths, deepcResult, mpcResult)
    validation = final_validate_motor_efforts(deepcResult, mpcResult);
    fid = fopen(fullfile(paths.reportsDir, '04_motor_effort_validity_report.md'), 'w');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# Motor Effort Validity Report\n\n');
    fprintf(fid, '1. DeePC does not output physical motor speeds in the SwarmDrones labyrinth branch. It outputs `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]`.\n');
    fprintf(fid, '2. MPC also outputs `[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]` in the final branch comparison.\n');
    fprintf(fid, '3. Because neither controller outputs physical motors here, the final figure uses equivalent normalized command efforts.\n');
    fprintf(fid, '4. The mapping is a documented linear allocation applied identically to DeePC and MPC.\n');
    fprintf(fid, '5. The resulting channels are not physical rad/s, but all four are non-placeholder channels.\n');
    fprintf(fid, '6. Motor 4 was previously suspicious because direct command-channel plotting could leave yaw-rate demand near zero; the new mapping uses all four command axes in all four equivalent channels.\n');
    fprintf(fid, '7. Valid for article as normalized equivalent effort: `%d`.\n\n', validation.validForArticle);
    fprintf(fid, 'Label to use: `Equivalent normalized motor effort`.\n');
end

function printRow(fid, scenarioLabel, result)
    fprintf(fid, '| %s | %s | %.6f | %.6f | %.6f | %.6f |\n', scenarioLabel, result.controller, ...
        result.metrics.RMSE_total, result.metrics.min_h, result.metrics.control_energy_total, ...
        result.metrics.mean_solver_time);
end

function name = betterRmse(a, b)
    if a.metrics.RMSE_total <= b.metrics.RMSE_total
        name = a.controller;
    else
        name = b.controller;
    end
end

function name = betterSafety(a, b)
    if a.metrics.min_h >= b.metrics.min_h
        name = a.controller;
    else
        name = b.controller;
    end
end

function name = betterEnergy(a, b)
    if a.metrics.control_energy_total <= b.metrics.control_energy_total
        name = a.controller;
    else
        name = b.controller;
    end
end

function name = heavierSolver(a, b)
    if a.metrics.mean_solver_time >= b.metrics.mean_solver_time
        name = a.controller;
    else
        name = b.controller;
    end
end
