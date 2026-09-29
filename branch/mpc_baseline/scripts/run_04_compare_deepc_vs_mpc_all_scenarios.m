function run_04_compare_deepc_vs_mpc_all_scenarios()
%RUN_04_COMPARE_DEEPC_VS_MPC_ALL_SCENARIOS Compare branch DeePC and branch-side MPC results.

    paths = run_00_setup_mpc_branch_paths();

    deepcBaseline = loadResultStruct(paths.baselineResultFile);
    deepcSoft = loadResultStruct(paths.softResultFile);
    deepcIterative = loadResultStruct(paths.iterativeResultFile);
    deepcTuned = [];
    if isfile(paths.tunedIterativeResultFile)
        deepcTuned = loadResultStruct(paths.tunedIterativeResultFile);
    end

    mpcBaseline = loadResultStruct(fullfile(paths.resultsDir, 'mpc_baseline_results.mat'));
    Sobs = load(fullfile(paths.resultsDir, 'mpc_obstacle_results.mat'), 'obstacleResults');
    mpcSoft = Sobs.obstacleResults.softResult;
    mpcIterative = Sobs.obstacleResults.iterativeResult;

    comparison = struct();
    comparison.deepcBaseline = deepcBaseline;
    comparison.deepcSoft = deepcSoft;
    comparison.deepcIterative = deepcIterative;
    comparison.deepcTuned = deepcTuned;
    comparison.mpcBaseline = mpcBaseline;
    comparison.mpcSoft = mpcSoft;
    comparison.mpcIterative = mpcIterative;

    save(fullfile(paths.resultsDir, 'deepc_mpc_comparison_results.mat'), 'comparison');

    rows = table();
    rows = [rows; resultToRow(deepcBaseline, 'Scenario 0 baseline', 'DeePC baseline', 'DeePC', 'baseline', 0)]; %#ok<AGROW>
    rows = [rows; resultToRow(deepcSoft, 'Scenario 1 obstacle', 'DeePC soft obstacle', 'DeePC', 'soft_linearized', getDeepcRho(deepcSoft))]; %#ok<AGROW>
    rows = [rows; resultToRow(deepcIterative, 'Scenario 2 obstacle', 'DeePC iterative obstacle', 'DeePC', 'iterative_linearized', getDeepcRho(deepcIterative))]; %#ok<AGROW>
    if ~isempty(deepcTuned)
        rows = [rows; resultToRow(deepcTuned, 'Scenario 2 obstacle', 'DeePC iterative tuned', 'DeePC', 'iterative_linearized', getDeepcRho(deepcTuned))]; %#ok<AGROW>
    end
    rows = [rows; resultToRow(mpcBaseline, 'Scenario 0 baseline', 'MPC baseline', 'MPC', 'baseline', 0)]; %#ok<AGROW>
    rows = [rows; resultToRow(mpcSoft, 'Scenario 1 obstacle', 'MPC soft obstacle', 'MPC', 'soft_linearized', getMpcRho(mpcSoft))]; %#ok<AGROW>
    rows = [rows; resultToRow(mpcIterative, 'Scenario 2 obstacle', 'MPC iterative obstacle', 'MPC', 'iterative_linearized', getMpcRho(mpcIterative))]; %#ok<AGROW>

    writetable(rows, fullfile(paths.tablesDir, 'deepc_mpc_comparison_metrics.csv'));

    paperRows = rows(:, {'method', 'scenario_name', 'RMSE_total', 'min_h', ...
        'num_safety_violations', 'control_energy_total', 'mean_solver_time', 'feasibility_rate'});
    paperRows.Properties.VariableNames = {'method', 'scenario', 'RMSE_total', 'min_h', ...
        'num_safety_violations', 'control_energy_total', 'mean_solver_time', 'feasibility_rate'};
    writetable(paperRows, fullfile(paths.tablesDir, 'paper_comparison_table.csv'));

    summaryRows = rows(:, {'scenario_name', 'method', 'RMSE_total', 'min_h', ...
        'num_safety_violations', 'max_violation_depth', 'mean_solver_time', 'feasibility_rate'});
    writetable(summaryRows, fullfile(paths.tablesDir, 'scenario_metrics_summary.csv'));

    allMetrics = struct();
    allMetrics.table = rows;
    save(fullfile(paths.resultsDir, 'all_metrics.mat'), 'allMetrics');

    scenario = deepcBaseline.scenario;
    compareScenario(paths, 'scenario_00', scenario, deepcBaseline, mpcBaseline, []);
    compareScenario(paths, 'scenario_01', scenario, deepcSoft, mpcSoft, []);
    compareScenario(paths, 'scenario_02', scenario, deepcIterative, mpcIterative, deepcTuned);
end

function compareScenario(paths, scenarioName, scenario, resultA, resultB, resultC)
    outputDir = fullfile(paths.comparisonFiguresDir, scenarioName);
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    series = [packSeries(resultA, [0.15, 0.40, 0.82], '-'), ...
        packSeries(resultB, [0.82, 0.20, 0.18], '--')];
    if ~isempty(resultC)
        series(end + 1) = packSeries(resultC, [0.15, 0.65, 0.30], '-.'); %#ok<AGROW>
    end

    mpc_generate_comparison_figures(outputDir, scenarioName, scenario, resultA.log.ref', series);
end

function result = loadResultStruct(filePath)
    S = load(filePath);
    if isfield(S, 'result')
        result = S.result;
    else
        names = fieldnames(S);
        result = S.(names{1});
    end
end

function rho = getDeepcRho(result)
    if isfield(result, 'obstacleOptions') && isfield(result.obstacleOptions, 'RhoObs')
        rho = result.obstacleOptions.RhoObs;
    else
        rho = 0;
    end
end

function rho = getMpcRho(result)
    if isfield(result, 'obstacleOptions') && isfield(result.obstacleOptions, 'RhoSlack')
        rho = result.obstacleOptions.RhoSlack;
    else
        rho = 0;
    end
end

function T = resultToRow(result, scenarioName, methodName, controllerType, obstacleMode, rhoObs)
    metrics = normalizeMetrics(result, controllerType);
    T = table();
    T.scenario_name = string(scenarioName);
    T.method = string(methodName);
    T.controller_type = string(controllerType);
    T.obstacle_mode = string(obstacleMode);
    T.N = getFieldOrNaN(result, {'controllerParams', 'N'}, {'configSummary', 'T_f'});
    T.Tini = getFieldOrNaN(result, {'controllerParams', 'Tini'}, {'configSummary', 'T_ini'});
    T.lambda_g = getFieldOrNaN(result, {'controllerParams', 'lambda_g'}, {'configSummary', 'lambda_g'});
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
    T.notes = string(buildNotes(result, controllerType));
end

function metrics = normalizeMetrics(result, controllerType)
    if strcmpi(controllerType, 'MPC')
        metrics = result.metrics;
        return;
    end

    data = struct();
    data.pos = result.log.pos;
    data.ref = result.log.ref;
    data.u = result.log.u;
    data.time = result.log.time;
    data.solveTime = result.log.solveTime;
    data.success = result.log.success;
    metrics = mpc_compute_article_metrics(data, result.scenario);
end

function notes = buildNotes(result, controllerType)
    if strcmpi(controllerType, 'DeePC')
        notes = 'Existing branch DeePC result.';
    elseif strcmpi(result.controllerMode, 'baseline')
        notes = 'Branch-side MPC baseline on the original labyrinth plant.';
    else
        notes = 'Branch-side obstacle-aware MPC with linearized slack constraints.';
    end
end

function value = getFieldOrNaN(result, primaryPath, fallbackPath)
    value = tryGetNested(result, primaryPath);
    if isnan(value)
        value = tryGetNested(result, fallbackPath);
    end
end

function value = tryGetNested(s, pathSpec)
    value = NaN;
    if isempty(pathSpec)
        return;
    end
    current = s;
    for i = 1:numel(pathSpec)
        key = pathSpec{i};
        if ~isstruct(current) || ~isfield(current, key)
            return;
        end
        current = current.(key);
    end
    if isnumeric(current) && isscalar(current)
        value = current;
    end
end

function series = packSeries(result, colorValue, lineStyle)
    series = struct();
    if isfield(result, 'controllerType') && strcmpi(result.controllerType, 'MPC')
        series.name = result.label;
    else
        series.name = result.label;
    end
    series.traj = result.log.pos';
    series.u = result.log.u;
    if isfield(result.log, 'euler')
        series.euler = result.log.euler;
    else
        series.euler = nan(3, numel(result.log.time));
    end
    series.time = result.log.time;
    if isfield(result, 'safetyMargin')
        series.safetyMargin = result.safetyMargin(:)';
    else
        series.safetyMargin = nan(1, numel(result.log.time));
    end
    series.trackingError = result.log.err;
    series.color = colorValue;
    series.lineStyle = lineStyle;
end
