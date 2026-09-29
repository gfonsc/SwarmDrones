% demo_deepc_labyrinth_database.m
% Uses a labyrinth-specific offline database with the real DeePC QP loop.

clear; clc; close all;

baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(baseDir, 'controllers', 'deepc'));
addpath(fullfile(baseDir, 'database'));
addpath(fullfile(baseDir, 'models'));

resultsDir = fullfile(baseDir, 'examples', 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% Offline database and DeePC setup
T_ini = 4;
T_f = 8;

fprintf('Loading labyrinth DeePC database...\n');
[U_train, Y_train, ~, meta] = loadLabyrinthData('Regenerate', true, ...
    'NumSamples', 360, 'Seed', 11);
scenario = meta.scenario;
cfg = scenario.model;

m = size(U_train, 1);
p = size(Y_train, 1);

uOffset = mean(U_train, 2);
yOffset = mean(Y_train, 2);
uScale = max(max(abs(U_train - uOffset), [], 2), [0.25; 0.25; 0.18; 0.30]);
yScale = max(max(abs(Y_train - yOffset), [], 2), [0.60; 0.60; 0.30; 0.25; 0.25; 0.18]);

U_lti = (U_train - uOffset) ./ uScale;
Y_lti = (Y_train - yOffset) ./ yScale;

params = struct();
params.T_ini = T_ini;
params.T_f = T_f;
params.lambda_y = 700;
params.lambda_g = 12;
params.qNorm = 2;
params.epsReg = 1e-7;
params.Q = diag([850, 850, 980, 8, 8, 10]);
params.R = diag([0.018, 0.018, 0.030, 0.10]);
params.u_r = (zeros(m, 1) - uOffset) ./ uScale;
params.y_r = (zeros(p, 1) - yOffset) ./ yScale;
params.u_min = (cfg.uMin - uOffset) ./ uScale;
params.u_max = (cfg.uMax - uOffset) ./ uScale;

yMinPhys = [scenario.bounds.x(1); scenario.bounds.y(1); scenario.bounds.z(1); ...
    cfg.uMin(1:3)];
yMaxPhys = [scenario.bounds.x(2); scenario.bounds.y(2); scenario.bounds.z(2); ...
    cfg.uMax(1:3)];
params.y_min = (yMinPhys - yOffset) ./ yScale;
params.y_max = (yMaxPhys - yOffset) ./ yScale;

fprintf('Building DeePC model for labyrinth envelope...\n');
model = deepcSetup(U_lti, Y_lti, params);

%% Closed-loop labyrinth mission
if isfield(scenario, 'trainingSetpoints') && ~isempty(scenario.trainingSetpoints)
    path = resamplePolyline3D(scenario.trainingSetpoints, 0.16);
else
    path = resamplePolyline3D(scenario.setpoints, 0.16);
end
maxSteps = 280;
dt = scenario.dt;

state = [scenario.A(:); 0; 0; 0; pi/2; 0];
y0 = selectOutput(state, scenario);
uZero = (zeros(m, 1) - uOffset) ./ uScale;
yNorm = (y0 - yOffset) ./ yScale;

uHist = repmat(uZero, 1, T_ini);
yHist = repmat(yNorm, 1, T_ini);

log = initLog(maxSteps, m, p);
pathIdx = 1;

solverOpts = struct('solver', 'auto', 'verbose', false);

fprintf('Starting DeePC labyrinth closed-loop simulation...\n');
for k = 1:maxSteps
    pos = state(1:3)';
    pathIdx = advancePathIndex(path, pathIdx, pos, 0.26);

    r = buildReferenceHorizon(path, pathIdx, T_f, dt, p, yOffset, yScale);
    [uOpt, yPred, info] = deepcAlgorithm(model, uHist(:), yHist(:), r, solverOpts);

    if info.success && all(isfinite(uOpt))
        uNorm = uOpt(1:m);
    else
        uNorm = uZero;
    end

    uCmd = uNorm .* uScale + uOffset;
    uCmd = max(cfg.uMin, min(cfg.uMax, uCmd));
    [state, yFull, uApplied] = simulateLabyrinthDroneStep(state, uCmd, cfg);

    yPhys = selectOutput(yFull, scenario);
    yNorm = (yPhys - yOffset) ./ yScale;
    uNormApplied = (uApplied - uOffset) ./ uScale;

    uHist = [uHist(:, 2:end), uNormApplied];
    yHist = [yHist(:, 2:end), yNorm];

    refPhys = path(pathIdx, :)';
    log.time(k) = (k - 1) * dt;
    log.pos(:, k) = yPhys(1:3);
    log.vel(:, k) = yPhys(4:6);
    log.ref(:, k) = refPhys;
    log.u(:, k) = uApplied;
    log.success(k) = info.success;
    log.pred(:, k) = denormalizePrediction(yPred, yOffset, yScale, p);
    log.pathIdx(k) = pathIdx;
    log.err(k) = norm(yPhys(1:3) - refPhys);

    if mod(k, 25) == 0 || k == 1
        fprintf('Time: %.1f s | Pos: [%.2f %.2f %.2f] | Ref: [%.2f %.2f %.2f] | QP: %d\n', ...
            log.time(k), yPhys(1), yPhys(2), yPhys(3), ...
            refPhys(1), refPhys(2), refPhys(3), info.success);
    end

    if pathIdx >= size(path, 1) && norm(yPhys(1:3) - scenario.B(:)) < 0.35
        log = trimLog(log, k);
        break;
    end
end

result = buildResult(log, path, scenario, meta, model);
validateResult(result);

fig2d = plotLabyrinth2D(result);
fig3d = plotLabyrinth3D(result);

png2d = fullfile(resultsDir, 'LabyrinthDeePCDatabase_2d.png');
png3d = fullfile(resultsDir, 'LabyrinthDeePCDatabase_3d.png');
matFile = fullfile(resultsDir, 'LabyrinthDeePCDatabase_result.mat');

saveFigure(fig2d, png2d);
saveFigure(fig3d, png3d);
save(matFile, 'result');

fprintf('\n====================================================\n');
fprintf('  DeePC Labyrinth Database Test Complete\n');
fprintf('====================================================\n');
fprintf('  QP success rate:     %.1f%%\n', 100 * result.metrics.qpSuccessRate);
fprintf('  Position RMSE:       %.3f m\n', result.metrics.positionRmse);
fprintf('  Final error:         %.3f m\n', result.metrics.finalError);
fprintf('  Max position error:  %.3f m\n', result.metrics.maxError);
fprintf('  Min clearance:       %.3f m\n', result.metrics.minClearance);
fprintf('  Min wall clearance:  %.3f m\n', result.metrics.minWallClearance);
fprintf('  Hover reached:       %d\n', result.metrics.hoverReached);
fprintf('  Right bypass:        %d\n', result.metrics.usedRightBypass);
fprintf('  Altitude up:         %d\n', result.metrics.usedAltitudeUp);
fprintf('  Altitude down:       %d\n', result.metrics.usedAltitudeDown);
fprintf('  Left bypass:         %d\n', result.metrics.usedLeftBypass);
fprintf('  Ceiling respected:   %d\n', result.metrics.ceilingRespected);
fprintf('  Collision-free:      %d\n', result.metrics.collisionFree);
fprintf('  Saved 2D:            %s\n', png2d);
fprintf('  Saved 3D:            %s\n', png3d);
fprintf('  Saved MAT:           %s\n', matFile);

%% Simulation helpers
function y = selectOutput(stateOrOutput, scenario)
    y = stateOrOutput(scenario.outputIdx);
end

function log = initLog(n, m, p)
    log = struct();
    log.time = zeros(1, n);
    log.pos = zeros(3, n);
    log.vel = zeros(3, n);
    log.ref = zeros(3, n);
    log.u = zeros(m, n);
    log.success = false(1, n);
    log.pred = zeros(p, n);
    log.pathIdx = zeros(1, n);
    log.err = zeros(1, n);
end

function log = trimLog(log, n)
    fields = fieldnames(log);
    for i = 1:numel(fields)
        v = log.(fields{i});
        if isvector(v)
            log.(fields{i}) = v(1:n);
        else
            log.(fields{i}) = v(:, 1:n);
        end
    end
end

function idx = advancePathIndex(path, idx, pos, radius)
    lookaheadEnd = min(idx + 24, size(path, 1));
    window = path(idx:lookaheadEnd, :);
    dist = sqrt(sum((window - pos).^2, 2));
    [~, localIdx] = min(dist);
    idx = idx + localIdx - 1;

    while idx < size(path, 1) && norm(path(idx, :) - pos) < radius
        idx = idx + 1;
    end
end

function r = buildReferenceHorizon(path, idx, T_f, dt, p, yOffset, yScale)
    rMat = zeros(p, T_f);
    for j = 1:T_f
        idxJ = min(idx + j - 1, size(path, 1));
        posRef = path(idxJ, :)';
        velRef = pathVelocity(path, idxJ, dt);
        yRef = [posRef; velRef];
        rMat(:, j) = (yRef - yOffset) ./ yScale;
    end
    r = rMat(:);
end

function vel = pathVelocity(path, idx, dt)
    if idx >= size(path, 1)
        vel = zeros(3, 1);
        return;
    end
    step = path(min(idx + 1, size(path, 1)), :) - path(idx, :);
    vel = step(:) / max(dt, 1e-9);
    speed = norm(vel);
    if speed > 1.05
        vel = vel * (1.05 / speed);
    end
end

function yPredPhys = denormalizePrediction(yPred, yOffset, yScale, p)
    if isempty(yPred)
        yPredPhys = zeros(p, 1);
    else
        yPredPhys = yPred(1:p) .* yScale + yOffset;
    end
end

function result = buildResult(log, path, scenario, meta, model)
    result = struct();
    result.scenario = scenario;
    result.path = path;
    result.log = log;
    result.meta = meta;
    result.modelDims = model.dims;
    result.metrics = computeMetrics(log, scenario);
end

function metrics = computeMetrics(log, s)
    traj = log.pos';
    err = sqrt(sum((log.pos - log.ref).^2, 1));

    metrics = struct();
    metrics.qpSuccessRate = mean(log.success);
    metrics.positionRmse = sqrt(mean(err.^2));
    metrics.finalError = norm(log.pos(:, end) - s.B(:));
    metrics.maxError = max(err);
    metrics.minClearance = minObstacleClearance(traj, s);
    metrics.minWallClearance = labyrinthMinWallClearance(traj, s);
    metrics.hoverReached = any(traj(:, 3) >= 0.95 * s.zHover);
    metrics.usedRightBypass = any(traj(:, 1) > 0.24 & traj(:, 1) < 1.12 & ...
        traj(:, 2) > 2.0 & traj(:, 2) < 4.0);
    metrics.usedAltitudeUp = max(traj(:, 3)) > s.zHover + 0.30;
    metrics.usedAltitudeDown = min(traj(traj(:, 2) > 1.0, 3)) < s.zHover - 0.25;
    metrics.usedLeftBypass = any(traj(:, 1) < 5.86 & traj(:, 1) > 4.85 & ...
        traj(:, 2) > 2.2 & traj(:, 2) < 4.1);
    metrics.ceilingRespected = all(traj(:, 3) < s.zCeiling);
    metrics.floorRespected = all(traj(:, 3) >= -1e-9);
    metrics.collisionFree = metrics.minClearance > -0.02;
    metrics.wallCollisionFree = metrics.minWallClearance > 0.03;
end

function validateResult(result)
    m = result.metrics;
    assert(m.qpSuccessRate > 0.96, 'LabyrinthDeePC:QpFailures', ...
        'The DeePC QP did not solve reliably enough.');
    assert(m.finalError < 0.45, 'LabyrinthDeePC:GoalMissed', ...
        'The DeePC trajectory did not reach the final labyrinth point.');
    assert(m.positionRmse < 0.45, 'LabyrinthDeePC:TrackingError', ...
        'The DeePC tracking error is too high for this test.');
    assert(m.hoverReached, 'LabyrinthDeePC:NoHover', ...
        'The drone did not leave the ground and reach hover.');
    assert(m.usedRightBypass && m.usedLeftBypass, 'LabyrinthDeePC:NoLateralAvoidance', ...
        'The lateral avoidance maneuvers were not exercised.');
    assert(m.usedAltitudeUp && m.usedAltitudeDown, 'LabyrinthDeePC:NoAltitudeAvoidance', ...
        'The altitude avoidance maneuvers were not exercised.');
    assert(m.ceilingRespected && m.floorRespected, 'LabyrinthDeePC:AltitudeLimit', ...
        'The DeePC path violated the altitude envelope.');
    assert(m.collisionFree, 'LabyrinthDeePC:Collision', ...
        'The DeePC path collided with an obstacle.');
    assert(m.wallCollisionFree, 'LabyrinthDeePC:WallCollision', ...
        'The DeePC path collided with a labyrinth wall.');
end

function clearance = minObstacleClearance(traj, s)
    clearance = inf;
    for k = 1:size(traj, 1)
        p = traj(k, :);
        for i = 1:size(s.obstacles, 1)
            obs = s.obstacles(i, :);
            if p(3) >= obs(3) && p(3) <= obs(4)
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
    fig = figure('Name', 'Labyrinth DeePC Database 2D', ...
        'Position', [80 80 980 760], 'Color', [0.06 0.06 0.06]);
    hold on; grid on; axis equal;
    styleDarkAxes(gca);
    drawCorridorWalls2D(s);
    drawObstacles2D(s);
    plot(result.path(:, 1), result.path(:, 2), 'r--', ...
        'LineWidth', 1.5, 'DisplayName', 'Reference path');
    plot(result.log.pos(1, :), result.log.pos(2, :), 'b-', ...
        'LineWidth', 2.0, 'DisplayName', 'DeePC tracked path');
    plot(s.setpoints(:, 1), s.setpoints(:, 2), 'rx', ...
        'MarkerSize', 8, 'LineWidth', 1.6, 'DisplayName', 'Setpoints');
    plot(s.A(1), s.A(2), 'go', 'MarkerFaceColor', 'g', ...
        'MarkerSize', 8, 'DisplayName', 'A start');
    plot(s.B(1), s.B(2), 'ko', 'MarkerFaceColor', [0.1 0.1 0.1], ...
        'MarkerSize', 8, 'DisplayName', 'B exit');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Real DeePC QP tracking inside labyrinth database envelope');
    xlim(s.axis2D(1:2));
    ylim(s.axis2D(3:4));
    legend('Location', 'eastoutside');
end

function fig = plotLabyrinth3D(result)
    s = result.scenario;
    fig = figure('Name', 'Labyrinth DeePC Database 3D', ...
        'Position', [110 90 1050 760], 'Color', [0.06 0.06 0.06]);
    hold on; grid on; axis vis3d;
    styleDarkAxes(gca);
    drawCorridorWalls3D(s);
    drawObstacles3D(s);
    drawCeiling(s);
    plot3(result.path(:, 1), result.path(:, 2), result.path(:, 3), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference path');
    plot3(result.log.pos(1, :), result.log.pos(2, :), result.log.pos(3, :), ...
        'b-', 'LineWidth', 2.0, 'DisplayName', 'DeePC tracked path');
    plot3(s.setpoints(:, 1), s.setpoints(:, 2), s.setpoints(:, 3), ...
        'rx', 'MarkerSize', 8, 'LineWidth', 1.6, 'DisplayName', 'Setpoints');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D DeePC labyrinth: takeoff, hover, lateral and altitude avoidance');
    xlim(s.axis3D(1:2));
    ylim(s.axis3D(3:4));
    zlim(s.axis3D(5:6));
    view(42, 28);
    daspect([1 1 0.6]);
    legend('Location', 'eastoutside');
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
