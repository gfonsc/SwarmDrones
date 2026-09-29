function artifacts = mpc_generate_comparison_figures(outputDir, scenarioName, scenario, ref, methodSeries)
%MPC_GENERATE_COMPARISON_FIGURES Generate a standard comparison figure set.

    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    time = methodSeries(1).time(:)';
    controlSeries = repmat(struct('name', '', 'u', [], 'color', [], 'lineStyle', '-'), 1, numel(methodSeries));
    eulerSeries = repmat(struct('name', '', 'euler', [], 'color', [], 'lineStyle', '-'), 1, numel(methodSeries));

    for i = 1:numel(methodSeries)
        controlSeries(i).name = methodSeries(i).name;
        controlSeries(i).u = methodSeries(i).u;
        controlSeries(i).color = methodSeries(i).color;
        controlSeries(i).lineStyle = methodSeries(i).lineStyle;

        eulerSeries(i).name = methodSeries(i).name;
        eulerSeries(i).euler = methodSeries(i).euler;
        eulerSeries(i).color = methodSeries(i).color;
        eulerSeries(i).lineStyle = methodSeries(i).lineStyle;
    end

    artifacts = struct();
    artifacts.map2d = fullfile(outputDir, [scenarioName, '_2d_trajectory']);
    artifacts.map3d = fullfile(outputDir, [scenarioName, '_3d_trajectory']);
    artifacts.xyz = fullfile(outputDir, [scenarioName, '_xyz']);
    artifacts.euler = fullfile(outputDir, [scenarioName, '_euler']);
    artifacts.control = fullfile(outputDir, [scenarioName, '_control']);
    artifacts.safety = fullfile(outputDir, [scenarioName, '_safety_margin']);
    artifacts.error = fullfile(outputDir, [scenarioName, '_tracking_error']);

    mpc_plot_2d_trajectory_with_obstacles(scenario, methodSeries, artifacts.map2d, ...
        sprintf('%s: 2D trajectory comparison', scenarioName));
    mpc_plot_3d_trajectory_with_obstacles(scenario, methodSeries, artifacts.map3d, ...
        sprintf('%s: 3D trajectory comparison', scenarioName));
    mpc_plot_xyz(time, ref, methodSeries, artifacts.xyz, ...
        sprintf('%s: x, y, z tracking', scenarioName));
    mpc_plot_euler_angles(time, eulerSeries, artifacts.euler, ...
        sprintf('%s: yaw, pitch, roll', scenarioName));
    mpc_plot_control_efforts(time, controlSeries, artifacts.control, ...
        sprintf('%s: four control channels', scenarioName), ...
        {'vx\_cmd', 'vy\_cmd', 'vz\_cmd', 'yaw\_rate\_cmd'});

    fig = figure('Color', 'w', 'Position', [120, 120, 980, 760]);
    subplot(2, 1, 1);
    hold on; grid on;
    for i = 1:numel(methodSeries)
        tSeries = getSeriesTime(methodSeries(i), time, numel(methodSeries(i).safetyMargin));
        n = min(numel(tSeries), numel(methodSeries(i).safetyMargin));
        plot(tSeries(1:n), methodSeries(i).safetyMargin(1:n), 'LineWidth', 1.8, ...
            'Color', methodSeries(i).color, 'LineStyle', methodSeries(i).lineStyle, ...
            'DisplayName', methodSeries(i).name);
    end
    yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Unsafe boundary');
    xlabel('Time (s)');
    ylabel('h (m)');
    title('Safety margin');
    legend('Location', 'best');

    subplot(2, 1, 2);
    hold on; grid on;
    for i = 1:numel(methodSeries)
        tSeries = getSeriesTime(methodSeries(i), time, numel(methodSeries(i).trackingError));
        n = min(numel(tSeries), numel(methodSeries(i).trackingError));
        plot(tSeries(1:n), methodSeries(i).trackingError(1:n), 'LineWidth', 1.8, ...
            'Color', methodSeries(i).color, 'LineStyle', methodSeries(i).lineStyle, ...
            'DisplayName', methodSeries(i).name);
    end
    xlabel('Time (s)');
    ylabel('||e||_2 (m)');
    title('Tracking error norm');
    legend('Location', 'best');
    sgtitle(sprintf('%s: safety and tracking error', scenarioName), 'FontWeight', 'bold');
    savefig(fig, [artifacts.safety, '.fig']);
    exportgraphics(fig, [artifacts.safety, '.png'], 'Resolution', 200);
    close(fig);

    fig = figure('Color', 'w', 'Position', [120, 120, 980, 420]);
    hold on; grid on;
    for i = 1:numel(methodSeries)
        tSeries = getSeriesTime(methodSeries(i), time, numel(methodSeries(i).trackingError));
        n = min(numel(tSeries), numel(methodSeries(i).trackingError));
        plot(tSeries(1:n), methodSeries(i).trackingError(1:n), 'LineWidth', 1.8, ...
            'Color', methodSeries(i).color, 'LineStyle', methodSeries(i).lineStyle, ...
            'DisplayName', methodSeries(i).name);
    end
    xlabel('Time (s)');
    ylabel('||e||_2 (m)');
    title(sprintf('%s: tracking error norm', scenarioName), 'FontWeight', 'bold');
    legend('Location', 'best');
    savefig(fig, [artifacts.error, '.fig']);
    exportgraphics(fig, [artifacts.error, '.png'], 'Resolution', 200);
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
