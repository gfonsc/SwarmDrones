function final_plot_3d_comparison(comparison, outputFile)
%FINAL_PLOT_3D_COMPARISON Create the final 3D paper figure.

    fig = figure('Color', 'w', 'InvertHardcopy', 'off', 'Visible', 'off', ...
        'Position', [80, 80, 1320, 600]);
    tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    plotPanel(nexttile, comparison.scenarioA.deepc, comparison.scenarioA.mpc, false, 'Scenario A: no obstacle');
    plotPanel(nexttile, comparison.scenarioB.deepc, comparison.scenarioB.mpc, true, 'Scenario B: with obstacle');

    exportgraphics(fig, outputFile, 'Resolution', 300);
    close(fig);
end

function plotPanel(ax, deepcResult, mpcResult, showObstacles, titleText)
    hold(ax, 'on'); grid(ax, 'on'); view(ax, 40, 26);
    styleAxes3d(ax);
    ref = deepcResult.reference.position;
    drawLabyrinthWalls3d(ax, deepcResult.scenario);
    drawSetpoints3d(ax, deepcResult.scenario);
    if showObstacles
        drawObstacles3d(ax, deepcResult.scenario);
    end
    plot3(ax, ref(1, :), ref(2, :), ref(3, :), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Reference');
    plot3(ax, deepcResult.log.pos(1, :), deepcResult.log.pos(2, :), deepcResult.log.pos(3, :), ...
        'Color', [0.10, 0.36, 0.80], 'LineWidth', 2.0, 'DisplayName', 'DeePC');
    plot3(ax, mpcResult.log.pos(1, :), mpcResult.log.pos(2, :), mpcResult.log.pos(3, :), ...
        'Color', [0.82, 0.20, 0.18], 'LineWidth', 2.0, 'LineStyle', '--', 'DisplayName', 'MPC');
    scatter3(ax, ref(1, 1), ref(2, 1), ref(3, 1), 55, 'g', 'filled', 'DisplayName', 'Start');
    scatter3(ax, ref(1, end), ref(2, end), ref(3, end), 55, 'k', 'd', 'filled', 'DisplayName', 'Finish');
    xlabel(ax, 'x (m)', 'Color', 'k');
    ylabel(ax, 'y (m)', 'Color', 'k');
    zlabel(ax, 'z (m)', 'Color', 'k');
    title(ax, titleText, 'FontWeight', 'bold', 'Color', 'k');
    xlim(ax, deepcResult.scenario.axis3D(1:2));
    ylim(ax, deepcResult.scenario.axis3D(3:4));
    zlim(ax, deepcResult.scenario.axis3D(5:6));
    leg = legend(ax, 'Location', 'best');
    styleLegend(leg);
end

function drawLabyrinthWalls3d(ax, scenario)
    if ~isfield(scenario, 'wallSegments') || isempty(scenario.wallSegments)
        return;
    end
    wallColor = [0.22, 0.22, 0.22];
    walls = scenario.wallSegments;
    zTop = scenario.zCeiling;
    for i = 1:size(walls, 1)
        a = walls(i, 1:2);
        b = walls(i, 3:4);
        plot3(ax, [a(1), b(1)], [a(2), b(2)], [0, 0], '-', ...
            'Color', wallColor, 'LineWidth', 1.4, 'HandleVisibility', 'off');
        plot3(ax, [a(1), b(1)], [a(2), b(2)], [zTop, zTop], '-', ...
            'Color', wallColor, 'LineWidth', 1.2, 'HandleVisibility', 'off');
        plot3(ax, [a(1), a(1)], [a(2), a(2)], [0, zTop], ':', ...
            'Color', wallColor, 'LineWidth', 0.8, 'HandleVisibility', 'off');
        plot3(ax, [b(1), b(1)], [b(2), b(2)], [0, zTop], ':', ...
            'Color', wallColor, 'LineWidth', 0.8, 'HandleVisibility', 'off');
    end
    plot3(ax, nan, nan, nan, '-', 'Color', wallColor, 'LineWidth', 1.5, ...
        'DisplayName', 'Labyrinth walls');
end

function drawSetpoints3d(ax, scenario)
    if ~isfield(scenario, 'setpoints') || isempty(scenario.setpoints)
        return;
    end
    pts = scenario.setpoints;
    plot3(ax, pts(:, 1), pts(:, 2), pts(:, 3), 'o', 'Color', [0.48, 0.20, 0.72], ...
        'MarkerFaceColor', [0.80, 0.68, 0.92], 'MarkerSize', 4.0, ...
        'LineStyle', 'none', 'DisplayName', 'Setpoints');
end

function styleAxes3d(ax)
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', ...
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

function drawObstacles3d(ax, scenario)
    for i = 1:size(scenario.obstacles, 1)
        obs = scenario.obstacles(i, :);
        drawCylinder(ax, obs, [0.75, 0.35, 0.30], 0.18);
        safeObs = obs;
        safeObs(5) = obs(5) + scenario.droneRadius + scenario.safeDistance;
        drawCylinder(ax, safeObs, [0.92, 0.58, 0.10], 0.06);
    end
    plot3(ax, nan, nan, nan, 'Color', [0.55, 0.12, 0.10], 'LineWidth', 1.5, ...
        'DisplayName', 'Obstacle');
    plot3(ax, nan, nan, nan, 'Color', [0.92, 0.58, 0.10], 'LineWidth', 1.5, ...
        'DisplayName', 'Safety bound');
end

function drawCylinder(ax, obs, colorValue, faceAlphaValue)
    theta = linspace(0, 2 * pi, 48);
    z = [obs(3), obs(4)];
    [Theta, Z] = meshgrid(theta, z);
    X = obs(1) + obs(5) * cos(Theta);
    Y = obs(2) + obs(5) * sin(Theta);
    surf(ax, X, Y, Z, 'FaceColor', colorValue, 'FaceAlpha', faceAlphaValue, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
end
