function fig = mpc_plot_2d_trajectory_with_obstacles(scenario, series, outputBase, plotTitle)
%MPC_PLOT_2D_TRAJECTORY_WITH_OBSTACLES Plot a 2D map of the labyrinth trajectories.

    fig = figure('Color', 'w', 'Position', [100, 100, 980, 720]);
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');

    walls = labyrinthWallSegments(scenario);
    for i = 1:size(walls, 1)
        plot(ax, walls(i, [1 3]), walls(i, [2 4]), 'k-', 'LineWidth', 1.5);
    end

    for i = 1:size(scenario.obstacles, 1)
        obs = scenario.obstacles(i, :);
        drawCircle(ax, obs(1:2), obs(5), [0.80, 0.35, 0.35], 0.25, '--');
        drawCircle(ax, obs(1:2), obs(5) + scenario.droneRadius + scenario.safeDistance, ...
            [0.95, 0.65, 0.20], 0.12, '-');
    end

    for i = 1:numel(series)
        traj = coerceTrajectory(series(i).traj);
        plot(ax, traj(:, 1), traj(:, 2), 'LineWidth', 2.2, ...
            'Color', series(i).color, 'LineStyle', series(i).lineStyle, ...
            'DisplayName', series(i).name);
        scatter(ax, traj(1, 1), traj(1, 2), 60, series(i).color, 'filled', ...
            'HandleVisibility', 'off');
        scatter(ax, traj(end, 1), traj(end, 2), 70, series(i).color, 'd', 'filled', ...
            'HandleVisibility', 'off');
    end

    xlabel(ax, 'x (m)');
    ylabel(ax, 'y (m)');
    title(ax, plotTitle, 'FontWeight', 'bold');
    xlim(ax, scenario.axis2D(1:2));
    ylim(ax, scenario.axis2D(3:4));
    legend(ax, 'Location', 'best');

    saveFigure(fig, outputBase);
end

function drawCircle(ax, center, radius, colorValue, faceAlphaValue, lineStyle)
    theta = linspace(0, 2 * pi, 180);
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    patch(ax, x, y, colorValue, 'FaceAlpha', faceAlphaValue, ...
        'EdgeColor', colorValue, 'LineStyle', lineStyle, 'LineWidth', 1.2, ...
        'HandleVisibility', 'off');
end

function traj = coerceTrajectory(traj)
    if size(traj, 2) == 3
        return;
    end
    traj = traj';
end

function saveFigure(fig, outputBase)
    savefig(fig, [outputBase, '.fig']);
    exportgraphics(fig, [outputBase, '.png'], 'Resolution', 200);
end
