function run_03_mpc_labyrinth_obstacle_aware()
%RUN_03_MPC_LABYRINTH_OBSTACLE_AWARE Run soft and iterative obstacle-aware MPC on the branch labyrinth scenario.

    paths = run_00_setup_mpc_branch_paths();
    scenarioData = mpc_load_branch_scenario();
    model = mpc_adapt_tcc_model_to_labyrinth(scenarioData);
    params = model.defaultParams;
    params.N = 8;

    obstacleOptions = struct();
    obstacleOptions.IncludeWalls = false;
    obstacleOptions.IncludeObstacles = true;
    obstacleOptions.ActivationDistance = 0.60;
    obstacleOptions.MaxConstraintsPerStep = 2;
    obstacleOptions.SafeDistance = scenarioData.scenario.safeDistance;
    obstacleOptions.RhoSlack = 1e4;
    obstacleOptions.MaxIter = 4;
    obstacleOptions.Tolerance = 1e-3;

    softResult = mpc_run_labyrinth_rollout(scenarioData, model, params, ...
        'Label', 'mpc_soft_obstacle', 'Mode', 'soft', 'ObstacleOptions', obstacleOptions);
    iterativeResult = mpc_run_labyrinth_rollout(scenarioData, model, params, ...
        'Label', 'mpc_iterative_obstacle', 'Mode', 'iterative', 'ObstacleOptions', obstacleOptions);

    softResult.mappedControl = mpc_motor_mapping(softResult.log.u, 'Mode', 'direct_command');
    iterativeResult.mappedControl = mpc_motor_mapping(iterativeResult.log.u, 'Mode', 'direct_command');

    softResult.artifacts = generateSingleMethodFigures(paths.scenario01FiguresDir, 'scenario_01', softResult, 'MPC soft obstacle');
    iterativeResult.artifacts = generateSingleMethodFigures(paths.scenario02FiguresDir, 'scenario_02', iterativeResult, 'MPC iterative obstacle');

    obstacleResults = struct();
    obstacleResults.softResult = softResult;
    obstacleResults.iterativeResult = iterativeResult;
    obstacleResults.obstacleOptions = obstacleOptions;
    save(fullfile(paths.resultsDir, 'mpc_obstacle_results.mat'), 'obstacleResults');

    baselineFile = fullfile(paths.resultsDir, 'mpc_baseline_results.mat');
    metricsRows = table();
    if isfile(baselineFile)
        S = load(baselineFile, 'result');
        metricsRows = [metricsRows; buildMetricsTable(S.result, 'Scenario 0 baseline', 'MPC baseline', 'baseline', 0)]; %#ok<AGROW>
    end
    metricsRows = [metricsRows; buildMetricsTable(softResult, 'Scenario 1 obstacle', 'MPC soft obstacle', 'soft_linearized', obstacleOptions.RhoSlack)]; %#ok<AGROW>
    metricsRows = [metricsRows; buildMetricsTable(iterativeResult, 'Scenario 2 obstacle', 'MPC iterative obstacle', 'iterative_linearized', obstacleOptions.RhoSlack)]; %#ok<AGROW>
    writetable(metricsRows, fullfile(paths.tablesDir, 'mpc_metrics.csv'));
end

function artifacts = generateSingleMethodFigures(outputDir, scenarioName, result, labelName)
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    series = struct( ...
        'name', labelName, ...
        'traj', result.log.pos', ...
        'u', result.log.u, ...
        'euler', result.log.euler, ...
        'time', result.log.time, ...
        'safetyMargin', result.safetyMargin(:)', ...
        'trackingError', result.log.err, ...
        'color', [0.85, 0.25, 0.15], ...
        'lineStyle', '-');

    artifacts = mpc_generate_comparison_figures(outputDir, scenarioName, result.scenario, result.log.ref', series);
end

function T = buildMetricsTable(result, scenarioName, methodName, obstacleMode, rhoObs)
    metrics = result.metrics;
    T = table();
    T.scenario_name = string(scenarioName);
    T.method = string(methodName);
    T.controller_type = string('MPC');
    T.obstacle_mode = string(obstacleMode);
    T.N = result.controllerParams.N;
    T.Tini = NaN;
    T.lambda_g = NaN;
    T.rho_obs = rhoObs;
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
    T.notes = string('Branch-side MPC on the original labyrinth plant and geometry.');
end
