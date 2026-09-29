function final_plot_2d_comparison(comparison, outputFile)
%FINAL_PLOT_2D_COMPARISON Create the final 2D paper figure.

    fig = figure('Color', 'w', 'InvertHardcopy', 'off', 'Visible', 'off', ...
        'Position', [80, 80, 1320, 560]);
    tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    plotPanel(nexttile, comparison.scenarioA.deepc, comparison.scenarioA.mpc, false, 'Scenario A: no obstacle');
    plotPanel(nexttile, comparison.scenarioB.deepc, comparison.scenarioB.mpc, true, 'Scenario B: with obstacle');

    exportgraphics(fig, outputFile, 'Resolution', 300);
    close(fig);
end

function plotPanel(ax, deepcResult, mpcResult, showObstacles, titleText)
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    styleAxes(ax);
    ref = deepcResult.reference.position;
    drawLabyrinthWalls2d(ax, deepcResult.scenario);
    drawSetpoints2d(ax, deepcResult.scenario);
    if showObstacles
        drawObstacles2d(ax, deepcResult.scenario);
    end
    plot(ax, ref(1, :), ref(2, :), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Reference');
    plot(ax, deepcResult.log.pos(1, :), deepcResult.log.pos(2, :), ...
        'Color', [0.10, 0.36, 0.80], 'LineWidth', 2.0, 'DisplayName', 'DeePC');
    plot(ax, mpcResult.log.pos(1, :), mpcResult.log.pos(2, :), ...
        'Color', [0.82, 0.20, 0.18], 'LineWidth', 2.0, 'LineStyle', '--', 'DisplayName', 'MPC');
    scatter(ax, ref(1, 1), ref(2, 1), 55, 'g', 'filled', 'DisplayName', 'Start');
    scatter(ax, ref(1, end), ref(2, end), 55, 'k', 'd', 'filled', 'DisplayName', 'Finish');
    xlabel(ax, 'x (m)', 'Color', 'k');
    ylabel(ax, 'y (m)', 'Color', 'k');
    title(ax, titleText, 'FontWeight', 'bold', 'Color', 'k');
    xlim(ax, deepcResult.scenario.axis2D(1:2));
    ylim(ax, deepcResult.scenario.axis2D(3:4));
    leg = legend(ax, 'Location', 'best');
    styleLegend(leg);
end

function drawLabyrinthWalls2d(ax, scenario)
    if ~isfield(scenario, 'wallSegments') || isempty(scenario.wallSegments)
        return;
    end
    wallColor = [0.22, 0.22, 0.22];
    walls = scenario.wallSegments;
    for i = 1:size(walls, 1)
        plot(ax, walls(i, [1, 3]), walls(i, [2, 4]), '-', ...
            'Color', wallColor, 'LineWidth', 2.0, 'HandleVisibility', 'off');
    end
    plot(ax, nan, nan, '-', 'Color', wallColor, 'LineWidth', 2.0, ...
        'DisplayName', 'Labyrinth walls');
end

function drawSetpoints2d(ax, scenario)
    if ~isfield(scenario, 'setpoints') || isempty(scenario.setpoints)
        return;
    end
    pts = scenario.setpoints;
    plot(ax, pts(:, 1), pts(:, 2), 'o', 'Color', [0.48, 0.20, 0.72], ...
        'MarkerFaceColor', [0.80, 0.68, 0.92], 'MarkerSize', 4.5, ...
        'LineStyle', 'none', 'DisplayName', 'Setpoints');
end

function styleAxes(ax)
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.72, 0.72, 0.72], 'GridAlpha', 0.35, ...
        'FontSize', 10, 'LineWidth', 0.8, 'Box', 'on');
    try
        disableDefaultInteractivity(ax);
        ax.Toolbar.Visible = 'off';
    catch
    end
end

function styleLegend(leg)
    set(leg, 'Color', 'w', 'TextColor', 'k', 'EdgeColor', [0.35, 0.35, 0.35]);
end

function drawObstacles2d(ax, scenario)
    theta = linspace(0, 2 * pi, 160);
    for i = 1:size(scenario.obstacles, 1)
        obs = scenario.obstacles(i, :);
        safeR = obs(5) + scenario.droneRadius + scenario.safeDistance;
        patch(ax, obs(1) + obs(5) * cos(theta), obs(2) + obs(5) * sin(theta), ...
            [0.75, 0.35, 0.30], 'FaceAlpha', 0.22, 'EdgeColor', [0.55, 0.12, 0.10], ...
            'LineWidth', 1.0, 'HandleVisibility', 'off');
        plot(ax, obs(1) + safeR * cos(theta), obs(2) + safeR * sin(theta), ...
            'Color', [0.92, 0.58, 0.10], 'LineStyle', ':', 'LineWidth', 1.5, ...
            'HandleVisibility', 'off');
    end
    plot(ax, nan, nan, 'Color', [0.55, 0.12, 0.10], 'LineWidth', 1.5, ...
        'DisplayName', 'Obstacle');
    plot(ax, nan, nan, ':', 'Color', [0.92, 0.58, 0.10], 'LineWidth', 1.5, ...
        'DisplayName', 'Safety bound');
end
