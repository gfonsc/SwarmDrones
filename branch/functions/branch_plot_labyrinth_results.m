function artifacts = branch_plot_labyrinth_results(result, outputDir, prefix)
%BRANCH_PLOT_LABYRINTH_RESULTS Generate branch-side figures for labyrinth runs.

    if nargin < 3 || isempty(prefix)
        prefix = result.label;
    end
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    artifacts = struct();
    artifacts.traj2d = fullfile(outputDir, [prefix, '_2d.png']);
    artifacts.traj3d = fullfile(outputDir, [prefix, '_3d.png']);
    artifacts.trackingError = fullfile(outputDir, [prefix, '_tracking_error.png']);
    artifacts.inputs = fullfile(outputDir, [prefix, '_inputs.png']);
    artifacts.safety = fullfile(outputDir, [prefix, '_safety_margin.png']);

    fig2d = plotLabyrinth2D(result);
    saveFigure(fig2d, artifacts.traj2d);
    close(fig2d);

    fig3d = plotLabyrinth3D(result);
    saveFigure(fig3d, artifacts.traj3d);
    close(fig3d);

    figErr = plotTrackingError(result);
    saveFigure(figErr, artifacts.trackingError);
    close(figErr);

    figInputs = plotInputs(result);
    saveFigure(figInputs, artifacts.inputs);
    close(figInputs);

    figSafety = plotSafetyMargin(result);
    saveFigure(figSafety, artifacts.safety);
    close(figSafety);
end

function fig = plotLabyrinth2D(result)
    s = result.scenario;
    fig = figure('Name', [result.label, ' 2D'], ...
        'Position', [80 80 980 760], 'Color', [0.06 0.06 0.06]);
    hold on; grid on; axis equal;
    styleDarkAxes(gca);
    drawCorridorWalls2D(s);
    drawObstacles2D(s);
    plot(result.path(:, 1), result.path(:, 2), 'r--', ...
        'LineWidth', 1.5, 'DisplayName', 'Reference path');
    plot(result.log.pos(1, :), result.log.pos(2, :), 'b-', ...
        'LineWidth', 2.0, 'DisplayName', 'Tracked path');
    plot(s.setpoints(:, 1), s.setpoints(:, 2), 'rx', ...
        'MarkerSize', 7, 'LineWidth', 1.4, 'DisplayName', 'Setpoints');
    plot(s.A(1), s.A(2), 'go', 'MarkerFaceColor', 'g', ...
        'MarkerSize', 8, 'DisplayName', 'A start');
    plot(s.B(1), s.B(2), 'ko', 'MarkerFaceColor', [0.1 0.1 0.1], ...
        'MarkerSize', 8, 'DisplayName', 'B exit');
    xlabel('X (m)');
    ylabel('Y (m)');
    title(strrep([result.label, ' - 2D trajectory'], '_', ' '));
    xlim(s.axis2D(1:2));
    ylim(s.axis2D(3:4));
    legend('Location', 'eastoutside');
end

function fig = plotLabyrinth3D(result)
    s = result.scenario;
    fig = figure('Name', [result.label, ' 3D'], ...
        'Position', [100 90 1050 760], 'Color', [0.06 0.06 0.06]);
    hold on; grid on; axis vis3d;
    styleDarkAxes(gca);
    drawCorridorWalls3D(s);
    drawObstacles3D(s);
    drawCeiling(s);
    plot3(result.path(:, 1), result.path(:, 2), result.path(:, 3), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference path');
    plot3(result.log.pos(1, :), result.log.pos(2, :), result.log.pos(3, :), ...
        'b-', 'LineWidth', 2.0, 'DisplayName', 'Tracked path');
    plot3(s.setpoints(:, 1), s.setpoints(:, 2), s.setpoints(:, 3), ...
        'rx', 'MarkerSize', 7, 'LineWidth', 1.4, 'DisplayName', 'Setpoints');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(strrep([result.label, ' - 3D trajectory'], '_', ' '));
    xlim(s.axis3D(1:2));
    ylim(s.axis3D(3:4));
    zlim(s.axis3D(5:6));
    view(42, 28);
    daspect([1 1 0.6]);
    legend('Location', 'eastoutside');
end

function fig = plotTrackingError(result)
    fig = figure('Name', [result.label, ' tracking error'], ...
        'Position', [120 100 900 480], 'Color', 'w');
    plot(result.log.time, result.log.err, 'LineWidth', 1.8, 'Color', [0.10 0.35 0.90]);
    grid on;
    xlabel('Time (s)');
    ylabel('Position error (m)');
    title(strrep([result.label, ' - tracking error'], '_', ' '));
end

function fig = plotInputs(result)
    names = result.scenario.inputNames;
    fig = figure('Name', [result.label, ' inputs'], ...
        'Position', [140 100 980 720], 'Color', 'w');
    for i = 1:size(result.log.u, 1)
        subplot(size(result.log.u, 1), 1, i);
        plot(result.log.time, result.log.u(i, :), 'LineWidth', 1.5, ...
            'Color', [0.85 0.25 0.15]);
        grid on;
        ylabel(names{i});
        if i == 1
            title(strrep([result.label, ' - applied inputs'], '_', ' '));
        end
        if i == size(result.log.u, 1)
            xlabel('Time (s)');
        end
    end
end

function fig = plotSafetyMargin(result)
    fig = figure('Name', [result.label, ' safety'], ...
        'Position', [150 100 920 480], 'Color', 'w');
    plot(result.log.time, result.safetyMargin, 'LineWidth', 1.8, ...
        'Color', [0.00 0.55 0.55]);
    hold on;
    yline(0, '--', 'Unsafe boundary', 'Color', [0.85 0.15 0.15], 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Safety margin h (m)');
    title(strrep([result.label, ' - safety margin'], '_', ' '));
end

function styleDarkAxes(ax)
    set(ax, 'Color', [0.05 0.05 0.05], ...
        'XColor', [0.88 0.88 0.88], ...
        'YColor', [0.88 0.88 0.88], ...
        'ZColor', [0.88 0.88 0.88], ...
        'GridColor', [0.55 0.55 0.55], ...
        'MinorGridColor', [0.35 0.35 0.35]);
end

function drawCorridorWalls2D(s)
    wallColor = [0.0 0.85 0.95];
    walls = labyrinthWallSegments(s);
    for i = 1:size(walls, 1)
        plot(walls(i, [1 3]), walls(i, [2 4]), '-', ...
            'Color', wallColor, 'LineWidth', 2.0, 'HandleVisibility', 'off');
    end
    plot(nan, nan, '-', 'Color', wallColor, 'LineWidth', 2.0, ...
        'DisplayName', 'Labyrinth walls');
end

function drawCorridorWalls3D(s)
    wallColor = [0.0 0.85 0.95];
    walls = labyrinthWallSegments(s);
    for i = 1:size(walls, 1)
        drawWallSegment3D(walls(i, 1:2), walls(i, 3:4), s.zCeiling, wallColor);
    end
    plot3(nan, nan, nan, '-', 'Color', wallColor, 'LineWidth', 2.0, ...
        'DisplayName', 'Labyrinth walls');
end

function drawWallSegment3D(a, b, zTop, color)
    plot3([a(1), b(1)], [a(2), b(2)], [0, 0], '-', ...
        'Color', color, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot3([a(1), b(1)], [a(2), b(2)], [zTop, zTop], '-', ...
        'Color', color, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot3([a(1), a(1)], [a(2), a(2)], [0, zTop], ':', ...
        'Color', color, 'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot3([b(1), b(1)], [b(2), b(2)], [0, zTop], ':', ...
        'Color', color, 'LineWidth', 0.8, 'HandleVisibility', 'off');
end

function drawObstacles2D(s)
    th = linspace(0, 2*pi, 100);
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        safeR = obs(5) + s.droneRadius + s.safeDistance;
        plot(obs(1) + safeR * cos(th), obs(2) + safeR * sin(th), ...
            ':', 'Color', [0.95 0.65 0.05], 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
        fill(obs(1) + obs(5) * cos(th), obs(2) + obs(5) * sin(th), ...
            obstacleColor(obs(6)), 'FaceAlpha', 0.42, ...
            'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.0, ...
            'HandleVisibility', 'off');
    end
    plot(nan, nan, ':', 'Color', [0.95 0.65 0.05], ...
        'LineWidth', 1.2, 'DisplayName', 'Safe distance');
    plot(nan, nan, 'o', 'Color', [0.2 0.2 0.2], ...
        'MarkerFaceColor', [0.6 0.6 0.6], 'DisplayName', 'Obstacles');
end

function drawObstacles3D(s)
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        [x, y, z] = cylinder(obs(5), 36);
        z = obs(3) + z * (obs(4) - obs(3));
        surf(x + obs(1), y + obs(2), z, ...
            'FaceColor', obstacleColor(obs(6)), 'FaceAlpha', 0.45, ...
            'EdgeColor', [0.2 0.2 0.2], 'EdgeAlpha', 0.20, ...
            'HandleVisibility', 'off');
    end
    plot3(nan, nan, nan, 'o', 'Color', [0.2 0.2 0.2], ...
        'MarkerFaceColor', [0.6 0.6 0.6], 'DisplayName', 'Obstacles');
end

function c = obstacleColor(modeId)
    switch modeId
        case 2
            c = [0.25 0.70 0.95];
        case 3
            c = [0.95 0.55 0.20];
        otherwise
            c = [0.80 0.80 0.80];
    end
end

function drawCeiling(s)
    [x, y] = meshgrid(s.axis3D(1:2), s.axis3D(3:4));
    z = s.zCeiling * ones(size(x));
    surf(x, y, z, 'FaceColor', [0.20 0.45 0.95], ...
        'FaceAlpha', 0.10, 'EdgeAlpha', 0.10, ...
        'DisplayName', 'Height limit');
end

function saveFigure(fig, fileName)
    try
        exportgraphics(fig, fileName, 'Resolution', 170);
    catch
        saveas(fig, fileName);
    end
end
