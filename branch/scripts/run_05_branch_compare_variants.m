clear; clc; close all;

paths = run_00_setup_branch_paths();

baselineFile = fullfile(paths.resultsDir, 'baseline_results.mat');
softFile = fullfile(paths.resultsDir, 'soft_obstacle_results.mat');
iterFile = fullfile(paths.resultsDir, 'iterative_obstacle_results.mat');

assert(isfile(baselineFile), 'Missing baseline result file.');
assert(isfile(softFile), 'Missing soft obstacle result file.');
assert(isfile(iterFile), 'Missing iterative obstacle result file.');

baseline = loadResult(baselineFile);
soft = loadResult(softFile);
iterative = loadResult(iterFile);

rows = [
    buildMetricRow('Baseline DeePC', 'original_labyrinth', baseline, 0)
    buildMetricRow('Soft obstacle DeePC', 'original_labyrinth', soft, soft.obstacleOptions.RhoObs)
    buildMetricRow('Iterative obstacle DeePC', 'original_labyrinth', iterative, iterative.obstacleOptions.RhoObs)
];
metricsTable = struct2table(rows);

metricsCsv = fullfile(paths.tablesDir, 'branch_metrics.csv');
writetable(metricsTable, metricsCsv);

comparisonFile = fullfile(paths.resultsDir, 'comparison_results.mat');
save(comparisonFile, 'baseline', 'soft', 'iterative', 'metricsTable');

paperDir = paths.paperFiguresDir;
if ~exist(paperDir, 'dir')
    mkdir(paperDir);
end

saveFigure(plotTrajectoryComparison(baseline, soft, iterative), ...
    fullfile(paperDir, 'labyrinth_baseline_vs_obstacle_deepc.png'));
saveFigure(plotSafetyComparison(baseline, soft, iterative), ...
    fullfile(paperDir, 'safety_margin_comparison.png'));
saveFigure(plotIterativeConvergence(iterative), ...
    fullfile(paperDir, 'iterative_convergence.png'));
saveFigure(plotControlComparison(baseline, soft, iterative), ...
    fullfile(paperDir, 'control_inputs_comparison.png'));

fprintf('Saved metrics CSV: %s\n', metricsCsv);
fprintf('Saved comparison MAT: %s\n', comparisonFile);
fprintf('Saved paper-ready figures in: %s\n', paperDir);

function result = loadResult(fileName)
    S = load(fileName, 'result');
    result = S.result;
end

function row = buildMetricRow(methodName, scenarioName, result, rhoObs)
    row = struct();
    row.method = string(methodName);
    row.scenario = string(scenarioName);
    row.N = result.configSummary.T_f;
    row.T_ini = result.configSummary.T_ini;
    row.lambda_g = result.configSummary.lambda_g;
    row.rho_obs = rhoObs;
    row.RMSE_total = result.metrics.positionRmse;
    row.min_h = result.metrics.minSafetyMargin;
    row.num_safety_violations = result.metrics.numSafetyViolations;
    row.max_violation_depth = result.metrics.maxViolationDepth;
    row.mean_slack = result.metrics.meanSlack;
    row.max_slack = result.metrics.maxSlack;
    row.mean_solver_time = result.metrics.meanSolverTime;
    row.max_solver_time = result.metrics.maxSolverTime;
    row.feasibility_rate = result.metrics.feasibilityRate;
    row.min_obstacle_clearance = result.metrics.minObstacleClearance;
    row.min_wall_clearance = result.metrics.minWallClearance;
    row.final_error = result.metrics.finalError;
    row.notes = string(composeNote(result));
end

function note = composeNote(result)
    if strcmpi(result.controllerMode, 'baseline')
        note = 'Original branch baseline using read-only DeePC stack.';
    elseif strcmpi(result.controllerMode, 'soft')
        note = 'Obstacle-only soft linearized constraints with slack.';
    else
        note = 'Obstacle-only iterative relinearization with slack.';
    end
end

function fig = plotTrajectoryComparison(baseline, soft, iterative)
    s = baseline.scenario;
    fig = figure('Name', 'Trajectory comparison', ...
        'Position', [100 100 980 760], 'Color', 'w');
    hold on; grid on; axis equal;
    drawGeometry2D(s);
    plot(baseline.path(:, 1), baseline.path(:, 2), 'k--', ...
        'LineWidth', 1.4, 'DisplayName', 'Reference');
    plot(baseline.log.pos(1, :), baseline.log.pos(2, :), ...
        'Color', [0.15 0.35 0.90], 'LineWidth', 1.8, 'DisplayName', 'Baseline DeePC');
    plot(soft.log.pos(1, :), soft.log.pos(2, :), ...
        'Color', [0.90 0.40 0.10], 'LineWidth', 1.8, 'DisplayName', 'Soft obstacle DeePC');
    plot(iterative.log.pos(1, :), iterative.log.pos(2, :), ...
        'Color', [0.10 0.60 0.35], 'LineWidth', 1.8, 'DisplayName', 'Iterative obstacle DeePC');
    plot(s.A(1), s.A(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7, 'DisplayName', 'Start');
    plot(s.B(1), s.B(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 7, 'DisplayName', 'Goal');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Labyrinth DeePC Branch: baseline versus obstacle-aware variants');
    xlim(s.axis2D(1:2));
    ylim(s.axis2D(3:4));
    legend('Location', 'eastoutside');
end

function fig = plotSafetyComparison(baseline, soft, iterative)
    fig = figure('Name', 'Safety comparison', ...
        'Position', [120 120 980 520], 'Color', 'w');
    hold on; grid on;
    plot(baseline.log.time, baseline.safetyMargin, 'LineWidth', 1.6, ...
        'Color', [0.15 0.35 0.90], 'DisplayName', 'Baseline DeePC');
    plot(soft.log.time, soft.safetyMargin, 'LineWidth', 1.6, ...
        'Color', [0.90 0.40 0.10], 'DisplayName', 'Soft obstacle DeePC');
    plot(iterative.log.time, iterative.safetyMargin, 'LineWidth', 1.6, ...
        'Color', [0.10 0.60 0.35], 'DisplayName', 'Iterative obstacle DeePC');
    yline(0, '--', 'Unsafe boundary', 'Color', [0.75 0.15 0.15], 'LineWidth', 1.2);
    xlabel('Time (s)');
    ylabel('Safety margin h (m)');
    title('Safety margin comparison near the labyrinth obstacles');
    legend('Location', 'best');
end

function fig = plotIterativeConvergence(iterative)
    fig = figure('Name', 'Iterative convergence', ...
        'Position', [140 140 980 640], 'Color', 'w');
    subplot(2, 1, 1);
    plot(iterative.log.time, iterative.log.iterationCount, 'LineWidth', 1.5, ...
        'Color', [0.10 0.60 0.35]);
    grid on;
    ylabel('Iterations');
    title('Iterative obstacle DeePC convergence');

    subplot(2, 1, 2);
    plot(iterative.log.time, iterative.log.convergenceResidual, 'LineWidth', 1.5, ...
        'Color', [0.05 0.20 0.70]);
    grid on;
    xlabel('Time (s)');
    ylabel('Residual');
end

function fig = plotControlComparison(baseline, soft, iterative)
    names = baseline.scenario.inputNames;
    fig = figure('Name', 'Control comparison', ...
        'Position', [160 120 980 740], 'Color', 'w');
    for i = 1:size(baseline.log.u, 1)
        subplot(size(baseline.log.u, 1), 1, i);
        hold on; grid on;
        plot(baseline.log.time, baseline.log.u(i, :), 'LineWidth', 1.1, ...
            'Color', [0.15 0.35 0.90], 'DisplayName', 'Baseline');
        plot(soft.log.time, soft.log.u(i, :), 'LineWidth', 1.1, ...
            'Color', [0.90 0.40 0.10], 'DisplayName', 'Soft obstacle');
        plot(iterative.log.time, iterative.log.u(i, :), 'LineWidth', 1.1, ...
            'Color', [0.10 0.60 0.35], 'DisplayName', 'Iterative obstacle');
        ylabel(names{i});
        if i == 1
            title('Control inputs comparison');
        end
        if i == size(baseline.log.u, 1)
            xlabel('Time (s)');
            legend('Location', 'eastoutside');
        end
    end
end

function drawGeometry2D(s)
    wallColor = [0.0 0.80 0.90];
    walls = labyrinthWallSegments(s);
    for i = 1:size(walls, 1)
        plot(walls(i, [1 3]), walls(i, [2 4]), '-', ...
            'Color', wallColor, 'LineWidth', 1.8, 'HandleVisibility', 'off');
    end

    th = linspace(0, 2*pi, 120);
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        safeR = obs(5) + s.droneRadius + s.safeDistance;
        plot(obs(1) + safeR * cos(th), obs(2) + safeR * sin(th), ...
            ':', 'Color', [0.90 0.65 0.10], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        fill(obs(1) + obs(5) * cos(th), obs(2) + obs(5) * sin(th), ...
            [0.85 0.85 0.85], 'FaceAlpha', 0.35, 'EdgeColor', [0.25 0.25 0.25], ...
            'HandleVisibility', 'off');
    end
end

function saveFigure(fig, fileName)
    try
        exportgraphics(fig, fileName, 'Resolution', 180);
    catch
        saveas(fig, fileName);
    end
    close(fig);
end
