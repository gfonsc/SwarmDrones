function run_05_generate_paper_figures()
%RUN_05_GENERATE_PAPER_FIGURES Generate fixed-name paper-ready DeePC-vs-MPC figures.

    paths = run_00_setup_mpc_branch_paths();
    compFile = fullfile(paths.resultsDir, 'deepc_mpc_comparison_results.mat');
    if ~isfile(compFile)
        run_04_compare_deepc_vs_mpc_all_scenarios();
    end

    S = load(compFile, 'comparison');
    comparison = S.comparison;

    series = [ ...
        packSeries(comparison.deepcIterative, 'DeePC iterative', [0.15, 0.40, 0.82], '-'), ...
        packSeries(comparison.mpcIterative, 'MPC iterative', [0.82, 0.20, 0.18], '--')];

    if ~isempty(comparison.deepcTuned)
        series(end + 1) = packSeries(comparison.deepcTuned, 'DeePC tuned iterative', [0.15, 0.65, 0.30], '-.'); %#ok<AGROW>
    end

    scenario = comparison.deepcIterative.scenario;
    ref = comparison.deepcIterative.log.ref';
    time = comparison.deepcIterative.log.time;

    base2d = fullfile(paths.paperFiguresDir, 'paper_2d_trajectory_deepc_vs_mpc_obstacles');
    base3d = fullfile(paths.paperFiguresDir, 'paper_3d_trajectory_deepc_vs_mpc_obstacles');
    baseControl = fullfile(paths.paperFiguresDir, 'paper_control_efforts_4_motors_deepc_vs_mpc');
    baseXYZ = fullfile(paths.paperFiguresDir, 'paper_xyz_deepc_vs_mpc');
    baseEuler = fullfile(paths.paperFiguresDir, 'paper_euler_angles_deepc_vs_mpc');
    baseSafety = fullfile(paths.paperFiguresDir, 'paper_safety_margin_deepc_vs_mpc');
    baseError = fullfile(paths.paperFiguresDir, 'paper_tracking_error_deepc_vs_mpc');

    mpc_plot_2d_trajectory_with_obstacles(scenario, series, base2d, 'DeePC vs MPC with labyrinth obstacles');
    mpc_plot_3d_trajectory_with_obstacles(scenario, series, base3d, 'DeePC vs MPC with labyrinth obstacles');
    mpc_plot_xyz(time, ref, series, baseXYZ, 'Position tracking comparison');

    eulerSeries = repmat(struct('name', '', 'euler', [], 'color', [], 'lineStyle', '-'), 1, numel(series));
    controlSeries = repmat(struct('name', '', 'u', [], 'color', [], 'lineStyle', '-'), 1, numel(series));
    for i = 1:numel(series)
        eulerSeries(i).name = series(i).name;
        eulerSeries(i).euler = series(i).euler;
        eulerSeries(i).color = series(i).color;
        eulerSeries(i).lineStyle = series(i).lineStyle;
        controlSeries(i).name = series(i).name;
        controlSeries(i).u = series(i).u;
        controlSeries(i).color = series(i).color;
        controlSeries(i).lineStyle = series(i).lineStyle;
    end

    mpc_plot_euler_angles(time, eulerSeries, baseEuler, 'Yaw, pitch, roll comparison');
    mpc_plot_control_efforts(time, controlSeries, baseControl, ...
        'Four command-channel efforts', {'vx\_cmd', 'vy\_cmd', 'vz\_cmd', 'yaw\_rate\_cmd'});

    fig = figure('Color', 'w', 'Position', [120, 120, 980, 420]);
    hold on; grid on;
    for i = 1:numel(series)
        tSeries = getSeriesTime(series(i), time, numel(series(i).safetyMargin));
        n = min(numel(tSeries), numel(series(i).safetyMargin));
        plot(tSeries(1:n), series(i).safetyMargin(1:n), 'LineWidth', 1.8, ...
            'Color', series(i).color, 'LineStyle', series(i).lineStyle, ...
            'DisplayName', series(i).name);
    end
    yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Unsafe boundary');
    xlabel('Time (s)');
    ylabel('h (m)');
    title('Safety margin comparison', 'FontWeight', 'bold');
    legend('Location', 'best');
    savefig(fig, [baseSafety, '.fig']);
    exportgraphics(fig, [baseSafety, '.png'], 'Resolution', 200);
    close(fig);

    fig = figure('Color', 'w', 'Position', [120, 120, 980, 420]);
    hold on; grid on;
    for i = 1:numel(series)
        tSeries = getSeriesTime(series(i), time, numel(series(i).trackingError));
        n = min(numel(tSeries), numel(series(i).trackingError));
        plot(tSeries(1:n), series(i).trackingError(1:n), 'LineWidth', 1.8, ...
            'Color', series(i).color, 'LineStyle', series(i).lineStyle, ...
            'DisplayName', series(i).name);
    end
    xlabel('Time (s)');
    ylabel('||e||_2 (m)');
    title('Tracking error comparison', 'FontWeight', 'bold');
    legend('Location', 'best');
    savefig(fig, [baseError, '.fig']);
    exportgraphics(fig, [baseError, '.png'], 'Resolution', 200);
end

function series = packSeries(result, name, colorValue, lineStyle)
    series = struct();
    series.name = name;
    series.traj = result.log.pos';
    series.u = result.log.u;
    if isfield(result.log, 'euler')
        series.euler = result.log.euler;
    else
        series.euler = nan(3, numel(result.log.time));
    end
    series.time = result.log.time;
    series.safetyMargin = result.safetyMargin(:)';
    series.trackingError = result.log.err;
    series.color = colorValue;
    series.lineStyle = lineStyle;
end

function tSeries = getSeriesTime(series, fallbackTime, nSamples)
    if isfield(series, 'time') && ~isempty(series.time)
        tSeries = series.time(:)';
    else
        tSeries = fallbackTime(:)';
    end
    if numel(tSeries) < nSamples
        if numel(tSeries) >= 2
            dt = median(diff(tSeries));
        else
            dt = 1;
        end
        lastValue = tSeries(end);
        tSeries = [tSeries, lastValue + dt * (1:(nSamples - numel(tSeries)))];
    end
end
