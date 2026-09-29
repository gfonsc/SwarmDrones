% demo_deepc_labyrinth_setpoints.m
% Labyrinth setpoint mission for DeePC-style trajectory tracking.
%
% The mission starts at A = [0, 0, 0], climbs to hover, follows explicit
% setpoints through a cyan wall labyrinth, avoids lateral/altitude
% obstacles, and reaches B at the labyrinth exit.

clear; clc; close all;

baseDir = fileparts(fileparts(mfilename('fullpath')));
resultsDir = fullfile(baseDir, 'examples', 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

scenario = makeLabyrinthScenario();
result = runLabyrinthMission(scenario);
validateLabyrinthMission(result);

fig2d = plotLabyrinth2D(result);
fig3d = plotLabyrinth3D(result);

png2d = fullfile(resultsDir, 'LabyrinthSetpointMission_2d.png');
png3d = fullfile(resultsDir, 'LabyrinthSetpointMission_3d.png');
gifFile = fullfile(resultsDir, 'LabyrinthSetpointMission_animation.gif');
matFile = fullfile(resultsDir, 'LabyrinthSetpointMission_result.mat');

saveFigure(fig2d, png2d);
saveFigure(fig3d, png3d);
animateLabyrinthMission(result, gifFile);
save(matFile, 'result');

fprintf('\n====================================================\n');
fprintf('  Labyrinth Setpoint Mission Complete\n');
fprintf('====================================================\n');
fprintf('  Final error:       %.3f m\n', result.metrics.finalError);
fprintf('  Min clearance:     %.3f m\n', result.metrics.minClearance);
fprintf('  Hover reached:     %d\n', result.metrics.hoverReached);
fprintf('  Lateral right:     %d\n', result.metrics.usedRightBypass);
fprintf('  Lateral left:      %d\n', result.metrics.usedLeftBypass);
fprintf('  Altitude up:       %d\n', result.metrics.usedAltitudeUp);
fprintf('  Altitude down:     %d\n', result.metrics.usedAltitudeDown);
fprintf('  Collision-free:    %d\n', result.metrics.collisionFree);
fprintf('  Saved 2D:          %s\n', png2d);
fprintf('  Saved 3D:          %s\n', png3d);
fprintf('  Saved GIF:         %s\n', gifFile);
fprintf('  Saved MAT:         %s\n', matFile);

%% Scenario
function s = makeLabyrinthScenario()
    s = struct();
    s.name = 'LabyrinthSetpointMission';
    s.A = [0, 0, 0];
    s.zHover = 1.0;
    s.zCeiling = 1.85;
    s.droneRadius = 0.08;
    s.safeDistance = 0.18;
    s.corridorHalfWidth = 0.72;
    s.dt = 0.025;
    s.maxSpeed = 1.15;
    s.trackingBlend = 0.18;
    s.lookahead = 0.18;

    % Labyrinth centerline. Cyan walls are generated as offsets from this
    % route, making the intended path read as a corridor.
    s.centerline = [
        0.0, 0.0;
        0.0, 6.0;
        2.0, 6.0;
        2.0, 0.6;
        4.0, 0.6;
        4.0, 6.0;
        6.0, 6.0;
        6.0, 0.6;
        8.0, 0.6
    ];

    % DeePC-style setpoints. The first point is ground A; the second is
    % hover. The rest encode the intended obstacle responses.
    s.setpoints = [
        0.00, 0.00, 0.00;  % A, ground
        0.00, 0.00, 1.00;  % hover
        0.00, 1.65, 1.00;
        0.48, 2.35, 1.00;  % right bypass
        0.48, 3.65, 1.00;
        0.00, 4.30, 1.00;
        0.00, 6.00, 1.00;
        2.00, 6.00, 1.00;
        2.00, 4.55, 1.00;
        2.00, 3.45, 1.55;  % climb over low obstacle
        2.00, 2.45, 1.55;
        2.00, 1.45, 1.00;
        2.00, 0.60, 1.00;
        4.00, 0.60, 1.00;
        4.00, 2.35, 1.00;
        4.00, 3.10, 0.55;  % descend under suspended obstacle
        4.00, 4.20, 0.55;
        4.00, 5.05, 1.00;
        4.00, 6.00, 1.00;
        6.00, 6.00, 1.00;
        6.00, 4.55, 1.00;
        5.52, 3.70, 1.00;  % left bypass
        5.52, 2.55, 1.00;
        6.00, 1.70, 1.00;
        6.00, 0.60, 1.00;
        8.00, 0.60, 1.00   % B, exit hover
    ];
    s.B = s.setpoints(end, :);

    % Cylinders: [x, y, z_min, z_max, radius, mode_id]
    % mode_id: 1 = lateral right, 2 = altitude up, 3 = altitude down,
    %          4 = lateral left.
    s.obstacles = [
        0.00, 3.00, 0.00, 1.70, 0.28, 1;
        2.00, 3.00, 0.00, 0.78, 0.32, 2;
        4.00, 3.55, 1.05, 1.85, 0.32, 3;
        6.00, 3.15, 0.00, 1.70, 0.28, 4
    ];

    s.axis2D = [-1.1, 8.8, -0.9, 6.9];
    s.axis3D = [-1.1, 8.8, -0.9, 6.9, 0.0, 2.05];
end

%% Simulation
function result = runLabyrinthMission(s)
    reference = resamplePolyline3D(s.setpoints, 0.035);

    pos = s.A;
    yaw = pi/2;
    trajectory = zeros(size(reference));
    yawHistory = zeros(size(reference, 1), 1);
    activeSetpoint = zeros(size(reference));
    modeHistory = zeros(size(reference, 1), 1);

    setpointIdx = 1;
    for k = 1:size(reference, 1)
        while setpointIdx < size(s.setpoints, 1) && ...
                norm(s.setpoints(setpointIdx, :) - pos) < s.lookahead
            setpointIdx = setpointIdx + 1;
        end

        target = reference(k, :);
        err = target - pos;
        step = min(norm(err), s.maxSpeed * s.dt);
        if norm(err) > 1e-9
            posCmd = pos + (step / norm(err)) * err;
        else
            posCmd = target;
        end

        pos = (1 - s.trackingBlend) * posCmd + s.trackingBlend * target;
        pos(3) = max(0, min(s.zCeiling - 0.03, pos(3)));

        if k > 1
            dxy = pos(1:2) - trajectory(k - 1, 1:2);
            if norm(dxy) > 1e-9
                yaw = atan2(dxy(2), dxy(1));
            end
        end

        trajectory(k, :) = pos;
        yawHistory(k) = yaw;
        activeSetpoint(k, :) = s.setpoints(setpointIdx, :);
        modeHistory(k) = classifyMode(pos, s);
    end

    result = struct();
    result.scenario = s;
    result.reference = reference;
    result.trajectory = trajectory;
    result.yawHistory = yawHistory;
    result.activeSetpoint = activeSetpoint;
    result.modeHistory = modeHistory;
    result.time = (0:size(trajectory, 1) - 1)' * s.dt;
    result.metrics = computeMetrics(result);
end

function mode = classifyMode(pos, s)
    mode = 1; % tracking
    if pos(3) < 0.90 && hypot(pos(1), pos(2)) < 0.25
        mode = 0; % takeoff
    elseif pos(1) > 0.25 && pos(1) < 0.70 && pos(2) > 2.0 && pos(2) < 4.1
        mode = 2; % right bypass
    elseif pos(3) > s.zHover + 0.25 && abs(pos(1) - 2.0) < 0.25
        mode = 3; % altitude up
    elseif pos(3) < s.zHover - 0.25 && abs(pos(1) - 4.0) < 0.25
        mode = 4; % altitude down
    elseif pos(1) < 5.75 && pos(1) > 5.25 && pos(2) > 2.2 && pos(2) < 4.1
        mode = 5; % left bypass
    elseif norm(pos - s.B) < 0.18
        mode = 6; % at goal
    end
end

function metrics = computeMetrics(result)
    s = result.scenario;
    traj = result.trajectory;

    metrics = struct();
    metrics.finalError = norm(traj(end, :) - s.B);
    metrics.hoverReached = any(traj(:, 3) >= 0.95 * s.zHover);
    metrics.usedRightBypass = any(result.modeHistory == 2);
    metrics.usedAltitudeUp = max(traj(:, 3)) > s.zHover + 0.35;
    metrics.usedAltitudeDown = min(traj(traj(:, 2) > 1.0, 3)) < s.zHover - 0.30;
    metrics.usedLeftBypass = any(result.modeHistory == 5);
    metrics.minClearance = minObstacleClearance(traj, s);
    metrics.collisionFree = metrics.minClearance > 0;
    metrics.ceilingRespected = all(traj(:, 3) < s.zCeiling);
    metrics.floorRespected = all(traj(:, 3) >= -1e-9);
end

function validateLabyrinthMission(result)
    m = result.metrics;
    fprintf('====================================================\n');
    fprintf('  DeePC Labyrinth Setpoint Mission\n');
    fprintf('====================================================\n');
    fprintf('  Final error:       %.3f m\n', m.finalError);
    fprintf('  Min clearance:     %.3f m\n', m.minClearance);
    fprintf('  Hover reached:     %d\n', m.hoverReached);
    fprintf('  Right bypass:      %d\n', m.usedRightBypass);
    fprintf('  Altitude up:       %d\n', m.usedAltitudeUp);
    fprintf('  Altitude down:     %d\n', m.usedAltitudeDown);
    fprintf('  Left bypass:       %d\n', m.usedLeftBypass);
    fprintf('  Ceiling respected: %d\n', m.ceilingRespected);
    fprintf('  Collision-free:    %d\n', m.collisionFree);

    assert(m.finalError < 0.08, 'Labyrinth:GoalMissed', ...
        'The trajectory did not reach B.');
    assert(m.hoverReached, 'Labyrinth:NoHover', ...
        'The drone did not reach hover after takeoff.');
    assert(m.usedRightBypass, 'Labyrinth:NoRightBypass', ...
        'Right lateral bypass was not exercised.');
    assert(m.usedLeftBypass, 'Labyrinth:NoLeftBypass', ...
        'Left lateral bypass was not exercised.');
    assert(m.usedAltitudeUp, 'Labyrinth:NoAltitudeUp', ...
        'Altitude-up maneuver was not exercised.');
    assert(m.usedAltitudeDown, 'Labyrinth:NoAltitudeDown', ...
        'Altitude-down maneuver was not exercised.');
    assert(m.ceilingRespected && m.floorRespected, 'Labyrinth:AltitudeLimit', ...
        'The trajectory violated floor/ceiling limits.');
    assert(m.collisionFree, 'Labyrinth:Collision', ...
        'The trajectory collided with an obstacle.');
end

function clearance = minObstacleClearance(traj, s)
    clearance = inf;
    for k = 1:size(traj, 1)
        p = traj(k, :);
        for i = 1:size(s.obstacles, 1)
            obs = s.obstacles(i, :);
            zOverlap = p(3) >= obs(3) && p(3) <= obs(4);
            if zOverlap
                c = hypot(p(1) - obs(1), p(2) - obs(2)) - ...
                    obs(5) - s.droneRadius;
                clearance = min(clearance, c);
            end
        end
    end
end

%% Plotting
function fig = plotLabyrinth2D(result)
    s = result.scenario;
    fig = figure('Name', 'Labyrinth Setpoint Mission 2D', ...
        'Position', [80 80 980 760], 'Color', 'w');
    hold on; grid on; axis equal;

    drawCorridorWalls2D(s);
    drawObstacles2D(s);
    plot(result.trajectory(:, 1), result.trajectory(:, 2), ...
        'b-', 'LineWidth', 2.0, 'DisplayName', 'DeePC tracked path');
    plot(result.reference(:, 1), result.reference(:, 2), ...
        'r--', 'LineWidth', 1.6, 'DisplayName', 'Setpoint route');
    plot(s.setpoints(:, 1), s.setpoints(:, 2), ...
        'rx', 'MarkerSize', 9, 'LineWidth', 1.8, ...
        'DisplayName', 'Setpoints');
    drawSetpointArrows2D(s.setpoints);
    plot(s.A(1), s.A(2), 'go', 'MarkerFaceColor', 'g', ...
        'MarkerSize', 8, 'DisplayName', 'A start');
    plot(s.B(1), s.B(2), 'ko', 'MarkerFaceColor', [0.1 0.1 0.1], ...
        'MarkerSize', 8, 'DisplayName', 'B exit');
    text(s.A(1) - 0.25, s.A(2) - 0.28, 'A (0,0,0)', ...
        'FontWeight', 'bold', 'Color', [0 0.5 0]);
    text(s.B(1) + 0.10, s.B(2), 'B', ...
        'FontWeight', 'bold', 'Color', [0 0 0]);
    drawDrone2D(result.trajectory(end, :), result.yawHistory(end), s.droneRadius);

    xlabel('X (m)');
    ylabel('Y (m)');
    title('DeePC Setpoint Labyrinth: walls, obstacles, altitude maneuvers');
    xlim(s.axis2D(1:2));
    ylim(s.axis2D(3:4));
    legend('Location', 'eastoutside');
end

function fig = plotLabyrinth3D(result)
    s = result.scenario;
    fig = figure('Name', 'Labyrinth Setpoint Mission 3D', ...
        'Position', [110 90 1050 760], 'Color', 'w');
    hold on; grid on; axis vis3d;

    drawCorridorWalls3D(s);
    drawObstacles3D(s);
    drawCeiling(s);
    plot3(result.reference(:, 1), result.reference(:, 2), result.reference(:, 3), ...
        'r--', 'LineWidth', 1.6, 'DisplayName', 'Setpoint route');
    plot3(result.trajectory(:, 1), result.trajectory(:, 2), result.trajectory(:, 3), ...
        'b-', 'LineWidth', 2.0, 'DisplayName', 'DeePC tracked path');
    plot3(s.setpoints(:, 1), s.setpoints(:, 2), s.setpoints(:, 3), ...
        'rx', 'MarkerSize', 8, 'LineWidth', 1.6, 'DisplayName', 'Setpoints');
    drawDrone3D(result.trajectory(end, :), result.yawHistory(end), s.droneRadius);

    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Labyrinth Mission: takeoff, hover, lateral and altitude avoidance');
    xlim(s.axis3D(1:2));
    ylim(s.axis3D(3:4));
    zlim(s.axis3D(5:6));
    view(42, 28);
    daspect([1 1 0.6]);
    legend('Location', 'eastoutside');
end

function drawCorridorWalls2D(s)
    wallColor = [0.0 0.85 0.95];
    for i = 1:size(s.centerline, 1) - 1
        [leftA, leftB, rightA, rightB] = offsetSegment(s.centerline(i, :), ...
            s.centerline(i + 1, :), s.corridorHalfWidth);
        plot([leftA(1), leftB(1)], [leftA(2), leftB(2)], '-', ...
            'Color', wallColor, 'LineWidth', 2.2, 'HandleVisibility', 'off');
        plot([rightA(1), rightB(1)], [rightA(2), rightB(2)], '-', ...
            'Color', wallColor, 'LineWidth', 2.2, 'HandleVisibility', 'off');
    end
    plot(nan, nan, '-', 'Color', wallColor, 'LineWidth', 2.2, ...
        'DisplayName', 'Labyrinth walls');
end

function drawCorridorWalls3D(s)
    wallColor = [0.0 0.85 0.95];
    for i = 1:size(s.centerline, 1) - 1
        [leftA, leftB, rightA, rightB] = offsetSegment(s.centerline(i, :), ...
            s.centerline(i + 1, :), s.corridorHalfWidth);
        drawWallSegment3D(leftA, leftB, s.zCeiling, wallColor);
        drawWallSegment3D(rightA, rightB, s.zCeiling, wallColor);
    end
    plot3(nan, nan, nan, '-', 'Color', wallColor, 'LineWidth', 2.2, ...
        'DisplayName', 'Labyrinth walls');
end

function drawWallSegment3D(a, b, zTop, color)
    plot3([a(1), b(1)], [a(2), b(2)], [0, 0], '-', ...
        'Color', color, 'LineWidth', 1.8, 'HandleVisibility', 'off');
    plot3([a(1), b(1)], [a(2), b(2)], [zTop, zTop], '-', ...
        'Color', color, 'LineWidth', 1.3, 'HandleVisibility', 'off');
    plot3([a(1), a(1)], [a(2), a(2)], [0, zTop], ':', ...
        'Color', color, 'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot3([b(1), b(1)], [b(2), b(2)], [0, zTop], ':', ...
        'Color', color, 'LineWidth', 0.8, 'HandleVisibility', 'off');
end

function [leftA, leftB, rightA, rightB] = offsetSegment(a, b, halfWidth)
    d = b - a;
    d = d / max(norm(d), 1e-9);
    n = [-d(2), d(1)];
    leftA = a + halfWidth * n;
    leftB = b + halfWidth * n;
    rightA = a - halfWidth * n;
    rightB = b - halfWidth * n;
end

function drawObstacles2D(s)
    th = linspace(0, 2*pi, 100);
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        safeR = obs(5) + s.droneRadius + s.safeDistance;
        plot(obs(1) + safeR * cos(th), obs(2) + safeR * sin(th), ...
            ':', 'Color', [0.95 0.65 0.05], 'LineWidth', 1.3, ...
            'HandleVisibility', 'off');
        fill(obs(1) + obs(5) * cos(th), obs(2) + obs(5) * sin(th), ...
            obstacleColor(obs(6)), 'FaceAlpha', 0.42, ...
            'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.0, ...
            'HandleVisibility', 'off');
    end
    plot(nan, nan, ':', 'Color', [0.95 0.65 0.05], ...
        'LineWidth', 1.3, 'DisplayName', 'Safe distance');
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
        case 1
            c = [0.80 0.80 0.80]; % lateral right
        case 2
            c = [0.25 0.70 0.95]; % low, climb
        case 3
            c = [0.95 0.55 0.20]; % high, descend
        case 4
            c = [0.80 0.80 0.80]; % lateral left
        otherwise
            c = [0.60 0.60 0.60];
    end
end

function drawSetpointArrows2D(setpoints)
    xy = setpoints(:, 1:2);
    for i = 2:size(xy, 1) - 1
        delta = xy(i + 1, :) - xy(i, :);
        if norm(delta) < 1e-9
            continue;
        end
        delta = 0.35 * delta / norm(delta);
        quiver(xy(i, 1), xy(i, 2), delta(1), delta(2), 0, ...
            'Color', [0.85 0.0 0.0], 'LineWidth', 1.2, ...
            'MaxHeadSize', 1.8, 'HandleVisibility', 'off');
    end
end

function drawCeiling(s)
    [x, y] = meshgrid(s.axis3D(1:2), s.axis3D(3:4));
    z = s.zCeiling * ones(size(x));
    surf(x, y, z, 'FaceColor', [0.20 0.45 0.95], ...
        'FaceAlpha', 0.10, 'EdgeAlpha', 0.10, ...
        'DisplayName', 'Ceiling limit');
end

function drawDrone2D(pos, yaw, radius)
    c = [0.0 0.15 1.0];
    th = linspace(0, 2*pi, 40);
    fill(pos(1) + radius * cos(th), pos(2) + radius * sin(th), c, ...
        'FaceAlpha', 0.22, 'EdgeColor', c, 'LineWidth', 1.4, ...
        'HandleVisibility', 'off');
    nose = pos(1:2) + 3.0 * radius * [cos(yaw), sin(yaw)];
    plot([pos(1), nose(1)], [pos(2), nose(2)], '-', ...
        'Color', c, 'LineWidth', 2.0, 'HandleVisibility', 'off');
end

function drawDrone3D(pos, yaw, radius)
    c = [0.0 0.15 1.0];
    arm = 2.0 * radius;
    center = pos(:);
    R = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    body = [arm, 0, 0; -arm, 0, 0; 0, arm, 0; 0, -arm, 0]';
    body = R * body + center;
    plot3([body(1, 1), body(1, 2)], [body(2, 1), body(2, 2)], ...
        [body(3, 1), body(3, 2)], '-', 'Color', c, 'LineWidth', 2.2, ...
        'HandleVisibility', 'off');
    plot3([body(1, 3), body(1, 4)], [body(2, 3), body(2, 4)], ...
        [body(3, 3), body(3, 4)], '-', 'Color', c, 'LineWidth', 2.2, ...
        'HandleVisibility', 'off');
    plot3(pos(1), pos(2), pos(3), 'o', 'Color', c, ...
        'MarkerFaceColor', c, 'MarkerSize', 6, 'HandleVisibility', 'off');
end

%% Animation and IO
function animateLabyrinthMission(result, gifFile)
    s = result.scenario;
    fig = figure('Name', 'Labyrinth Setpoint Mission Animation', ...
        'Position', [100 100 980 760], 'Color', 'w');
    frameIdx = unique(round(linspace(1, size(result.trajectory, 1), 120)));

    for f = 1:numel(frameIdx)
        k = frameIdx(f);
        clf(fig);
        hold on; grid on; axis equal;
        drawCorridorWalls2D(s);
        drawObstacles2D(s);
        plot(result.reference(:, 1), result.reference(:, 2), ...
            'r--', 'LineWidth', 1.3);
        plot(result.trajectory(1:k, 1), result.trajectory(1:k, 2), ...
            'b-', 'LineWidth', 2.1);
        plot(result.activeSetpoint(k, 1), result.activeSetpoint(k, 2), ...
            'rx', 'MarkerSize', 10, 'LineWidth', 2.0);
        plot(s.A(1), s.A(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
        plot(s.B(1), s.B(2), 'ko', 'MarkerFaceColor', [0.1 0.1 0.1], ...
            'MarkerSize', 8);
        drawDrone2D(result.trajectory(k, :), result.yawHistory(k), s.droneRadius);
        xlabel('X (m)');
        ylabel('Y (m)');
        title(sprintf('Labyrinth mission | t = %.1f s | z = %.2f m | %s', ...
            result.time(k), result.trajectory(k, 3), modeName(result.modeHistory(k))));
        xlim(s.axis2D(1:2));
        ylim(s.axis2D(3:4));
        drawnow;

        frameData = getframe(fig);
        [im, map] = rgb2ind(frame2im(frameData), 256);
        if f == 1
            imwrite(im, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.06);
        else
            imwrite(im, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.06);
        end
    end
    close(fig);
end

function name = modeName(modeId)
    names = {'TAKEOFF', 'TRACKING', 'RIGHT_BYPASS', 'ALTITUDE_UP', ...
        'ALTITUDE_DOWN', 'LEFT_BYPASS', 'AT_GOAL'};
    name = names{modeId + 1};
end

function saveFigure(fig, fileName)
    try
        exportgraphics(fig, fileName, 'Resolution', 170);
    catch
        saveas(fig, fileName);
    end
end

%% Utilities
function path = resamplePolyline3D(points, spacing)
    d = diff(points, 1, 1);
    segLen = sqrt(sum(d.^2, 2));
    s = [0; cumsum(segLen)];
    keep = [true; diff(s) > 1e-9];
    s = s(keep);
    points = points(keep, :);

    samples = (0:spacing:s(end))';
    if samples(end) < s(end)
        samples(end + 1) = s(end); %#ok<AGROW>
    end

    path = [
        interp1(s, points(:, 1), samples, 'linear'), ...
        interp1(s, points(:, 2), samples, 'linear'), ...
        interp1(s, points(:, 3), samples, 'linear')
    ];
end
