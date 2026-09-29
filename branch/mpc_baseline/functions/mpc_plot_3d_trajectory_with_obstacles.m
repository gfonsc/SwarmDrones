function fig = mpc_plot_3d_trajectory_with_obstacles(scenario, series, outputBase, plotTitle)
%MPC_PLOT_3D_TRAJECTORY_WITH_OBSTACLES Plot a 3D trajectory with cylindrical obstacles.

    fig = figure('Color', 'w', 'Position', [120, 120, 980, 720]);
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    view(ax, 40, 26);

    for i = 1:size(scenario.obstacles, 1)
        drawCylinder(ax, scenario.obstacles(i, :), [0.85, 0.35, 0.35], 0.18);
        safeObs = scenario.obstacles(i, :);
        safeObs(5) = safeObs(5) + scenario.droneRadius + scenario.safeDistance;
        drawCylinder(ax, safeObs, [0.95, 0.70, 0.25], 0.08);
    end

    for i = 1:numel(series)
        traj = coerceTrajectory(series(i).traj);
        plot3(ax, traj(:, 1), traj(:, 2), traj(:, 3), 'LineWidth', 2.2, ...
            'Color', series(i).color, 'LineStyle', series(i).lineStyle, ...
            'DisplayName', series(i).name);
        scatter3(ax, traj(1, 1), traj(1, 2), traj(1, 3), 60, series(i).color, 'filled', ...
            'HandleVisibility', 'off');
        scatter3(ax, traj(end, 1), traj(end, 2), traj(end, 3), 70, series(i).color, 'd', 'filled', ...
            'HandleVisibility', 'off');
    end

    xlabel(ax, 'x (m)');
    ylabel(ax, 'y (m)');
    zlabel(ax, 'z (m)');
    title(ax, plotTitle, 'FontWeight', 'bold');
    xlim(ax, scenario.axis3D(1:2));
    ylim(ax, scenario.axis3D(3:4));
    zlim(ax, scenario.axis3D(5:6));
    legend(ax, 'Location', 'best');

    savefig(fig, [outputBase, '.fig']);
    exportgraphics(fig, [outputBase, '.png'], 'Resolution', 200);
end

function drawCylinder(ax, obs, colorValue, faceAlphaValue)
    theta = linspace(0, 2 * pi, 40);
    z = [obs(3), obs(4)];
    [Theta, Z] = meshgrid(theta, z);
    X = obs(1) + obs(5) * cos(Theta);
    Y = obs(2) + obs(5) * sin(Theta);
    surf(ax, X, Y, Z, 'FaceColor', colorValue, 'FaceAlpha', faceAlphaValue, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
end

function traj = coerceTrajectory(traj)
    if size(traj, 2) == 3
        return;
    end
    traj = traj';
end
