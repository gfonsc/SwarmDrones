function run_02_mpc_labyrinth_baseline()
%RUN_02_MPC_LABYRINTH_BASELINE Run the baseline branch-side MPC on the labyrinth path.

    paths = run_00_setup_mpc_branch_paths();
    scenarioData = mpc_load_branch_scenario();
    model = mpc_adapt_tcc_model_to_labyrinth(scenarioData);
    params = model.defaultParams;
    params.N = 8;

    result = mpc_run_labyrinth_rollout(scenarioData, model, params, ...
        'Label', 'mpc_baseline', 'Mode', 'baseline');

    result.mappedControl = mpc_motor_mapping(result.log.u, 'Mode', 'direct_command');
    scenarioName = 'scenario_00';
    result.artifacts = generateSingleMethodFigures(paths.scenario00FiguresDir, scenarioName, result);

    save(fullfile(paths.resultsDir, 'mpc_baseline_results.mat'), 'result');

    metricsTable = buildMetricsTable(result, 'Scenario 0 baseline', 'MPC baseline', 'baseline');
    writetable(metricsTable, fullfile(paths.tablesDir, 'mpc_metrics.csv'));
end

function artifacts = generateSingleMethodFigures(outputDir, scenarioName, result)
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    series = struct( ...
        'name', 'MPC baseline', ...
        'traj', result.log.pos', ...
        'u', result.log.u, ...
        'euler', result.log.euler, ...
        'time', result.log.time, ...
        'safetyMargin', result.safetyMargin(:)', ...
        'trackingError', result.log.err, ...
        'color', [0.10, 0.45, 0.85], ...
        'lineStyle', '-');

    artifacts = mpc_generate_comparison_figures(outputDir, scenarioName, result.scenario, result.log.ref', series);
end

function T = buildMetricsTable(result, scenarioName, methodName, obstacleMode)
    metrics = result.metrics;
    T = table();
    T.scenario_name = string(scenarioName);
    T.method = string(methodName);
    T.controller_type = string('MPC');
    T.obstacle_mode = string(obstacleMode);
    T.N = result.controllerParams.N;
    T.Tini = NaN;
    T.lambda_g = NaN;
    T.rho_obs = 0;
    T.RMSE_x = metrics.RMSE_x;
    T.RMSE_y = metrics.RMSE_y;
    T.RMSE_z = metrics.RMSE_z;
    T.RMSE_total = metrics.RMSE_total;
    T.MAE_x = metrics.MAE_x;
    T.MAE_y = metrics.MAE_y;
    T.MAE_z = metrics.MAE_z;
    T.max_error_total = metrics.max_error_total;
    T.final_error_total = metrics.final_error_total;
    T.control_energy_total = metrics.control_energy_total;
    T.control_variation_total = metrics.control_variation_total;
    T.min_distance_to_obstacle = metrics.min_distance_to_obstacle;
    T.min_h = metrics.min_h;
    T.num_safety_violations = metrics.num_safety_violations;
    T.max_violation_depth = metrics.max_violation_depth;
    T.mean_solver_time = metrics.mean_solver_time;
    T.max_solver_time = metrics.max_solver_time;
    T.feasibility_rate = metrics.feasibility_rate;
    T.notes = string('Baseline branch-side MPC on the original labyrinth plant.');
end
