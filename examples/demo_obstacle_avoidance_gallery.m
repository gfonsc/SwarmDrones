% demo_obstacle_avoidance_gallery.m
% Presentation-quality obstacle avoidance scenarios with static plots and GIFs.
%
% This demo is intentionally independent from the DeePC QP benchmark scale.
% It validates the obstacle/path-planning visual layer using clean scenarios
% similar to common navigation figures: circular obstacles and corridor gaps.

clear; clc; close all;

baseDir = fileparts(fileparts(mfilename('fullpath')));
resultsDir = fullfile(baseDir, 'examples', 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

fprintf('====================================================\n');
fprintf('  Obstacle Avoidance Scenario Gallery\n');
fprintf('====================================================\n');

scenarios = {makeCircularScenario(), makeCorridorScenario()};
results = cell(size(scenarios));

for i = 1:numel(scenarios)
    scenario = scenarios{i};
    fprintf('\n--- Scenario: %s ---\n', scenario.name);

    rawPath = planGridPath(scenario);
    refPath = smoothPathSafe(rawPath, scenario);
    result = simulateFollower(scenario, refPath);
    result.rawPath = rawPath;
    result.reference = refPath;

    validateScenario(result);
    saveScenarioResult(result, resultsDir);
    results{i} = result;
end

fprintf('\n====================================================\n');
fprintf('  Gallery Complete\n');
fprintf('====================================================\n');
for i = 1:numel(results)
    r = results{i};
    fprintf('  %s: PASS | final error %.3f m | min clearance %.3f m\n', ...
        r.name, r.finalError, r.minClearance);
end

%% Scenario definitions
function scenario = makeCircularScenario()
    scenario = struct();
    scenario.name = 'CircularObstacleField';
    scenario.kind = 'circles';
    scenario.xLim = [0, 8.4];
    scenario.yLim = [-0.2, 7.4];
    scenario.start = [0.25, 0.20];
    scenario.goal = [7.85, 3.35];
    scenario.vehicleRadius = 0.12;
    scenario.safeDistance = 0.28;
    scenario.gridResolution = 0.08;
    scenario.pathSpacing = 0.08;
    scenario.maxSteps = 1200;
    scenario.dt = 0.04;
    scenario.speed = 0.72;
    scenario.lookahead = 0.16;
    scenario.trackingGain = 3.2;
    scenario.title = 'Circular Obstacles: Reference vs Tracked Path';
    scenario.obstacles = [
        1.05, 1.70, 0.22;
        2.15, 4.00, 0.50;
        2.45, 0.80, 0.32;
        3.05, 2.05, 0.24;
        3.75, 7.05, 0.30;
        4.10, 3.15, 0.38;
        4.95, 2.45, 0.42;
        5.35, 5.20, 0.28;
        5.95, 4.10, 0.36;
        6.60, 6.35, 0.60;
        7.25, 1.90, 0.26;
        7.55, 5.65, 0.30
    ];
    scenario.preferredWaypoints = [
        0.25, 0.20;
        0.65, 1.10;
        0.55, 2.10;
        0.75, 3.55;
        1.20, 4.80;
        2.65, 5.15;
        3.90, 5.45;
        4.70, 6.10;
        5.35, 5.80;
        6.90, 4.60;
        7.05, 3.60;
        7.85, 3.35
    ];
end

function scenario = makeCorridorScenario()
    scenario = struct();
    scenario.name = 'CorridorDestinationMission';
    scenario.kind = 'corridor';
    scenario.xLim = [-2, 54];
    scenario.yLim = [-2, 32];
    scenario.start = [45.0, 2.3];
    scenario.goal = [7.5, 23.5];
    scenario.vehicleRadius = 0.55;
    scenario.safeDistance = 0.65;
    scenario.gridResolution = 0.45;
    scenario.pathSpacing = 0.35;
    scenario.maxSteps = 1400;
    scenario.dt = 0.05;
    scenario.speed = 2.10;
    scenario.lookahead = 0.65;
    scenario.trackingGain = 2.7;
    scenario.title = 'Corridor Gap Mission: Start to Destination';
    scenario.destination = [4.5, 20.0, 5.0, 8.5]; % [x y w h]
    scenario.freePolygon = [
        0, 18;
        23, 18;
        23, 1.0;
        47, 1.0;
        50, 8.5;
        38, 8.5;
        38, 20.5;
        31, 29.2;
        0, 29.2
    ];
    scenario.obstacles = zeros(0, 3);
    scenario.preferredWaypoints = [
        45.0, 2.3;
        43.2, 3.6;
        39.0, 6.4;
        34.0, 8.8;
        31.0, 11.4;
        27.6, 15.2;
        24.2, 18.8;
        18.0, 22.2;
        10.5, 23.0;
        7.5, 23.5
    ];
end

%% Planning
function path = planGridPath(scenario)
    if isfield(scenario, 'preferredWaypoints')
        preferred = resamplePath(scenario.preferredWaypoints, scenario.gridResolution);
        if ~pathIsFree(preferred, scenario)
            warning('ObstacleGallery:PreferredPathNearBoundary', ...
                'Preferred path for %s is close to an obstacle/boundary.', scenario.name);
        end
        path = preferred;
        return;
    end

    xVec = scenario.xLim(1):scenario.gridResolution:scenario.xLim(2);
    yVec = scenario.yLim(1):scenario.gridResolution:scenario.yLim(2);
    nx = numel(xVec);
    ny = numel(yVec);

    free = false(ny, nx);
    clearance = zeros(ny, nx);
    for iy = 1:ny
        for ix = 1:nx
            p = [xVec(ix), yVec(iy)];
            [free(iy, ix), clearance(iy, ix)] = isFreePoint(p, scenario);
        end
    end

    startIdx = nearestFreeIndex(scenario.start, xVec, yVec, free);
    goalIdx = nearestFreeIndex(scenario.goal, xVec, yVec, free);

    infCost = inf(ny, nx);
    gScore = infCost;
    fScore = infCost;
    openSet = false(ny, nx);
    closedSet = false(ny, nx);
    cameFrom = zeros(ny, nx);

    gScore(startIdx(1), startIdx(2)) = 0;
    fScore(startIdx(1), startIdx(2)) = heuristic(startIdx, goalIdx, xVec, yVec);
    openSet(startIdx(1), startIdx(2)) = true;

    dirs = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];

    while any(openSet(:))
        openScores = fScore;
        openScores(~openSet) = inf;
        [~, linearCurrent] = min(openScores(:));
        [cy, cx] = ind2sub(size(openScores), linearCurrent);

        if cy == goalIdx(1) && cx == goalIdx(2)
            path = reconstructPath(cameFrom, [cy, cx], xVec, yVec);
            path(1, :) = scenario.start;
            path(end, :) = scenario.goal;
            return;
        end

        openSet(cy, cx) = false;
        closedSet(cy, cx) = true;

        for i = 1:size(dirs, 1)
            nyi = cy + dirs(i, 1);
            nxi = cx + dirs(i, 2);
            if nyi < 1 || nyi > ny || nxi < 1 || nxi > nx
                continue;
            end
            if ~free(nyi, nxi) || closedSet(nyi, nxi)
                continue;
            end

            step = hypot(xVec(nxi) - xVec(cx), yVec(nyi) - yVec(cy));
            clearancePenalty = max(0, scenario.safeDistance - clearance(nyi, nxi));
            tentative = gScore(cy, cx) + step * (1 + 3.0 * clearancePenalty);

            if ~openSet(nyi, nxi)
                openSet(nyi, nxi) = true;
            elseif tentative >= gScore(nyi, nxi)
                continue;
            end

            cameFrom(nyi, nxi) = sub2ind([ny, nx], cy, cx);
            gScore(nyi, nxi) = tentative;
            fScore(nyi, nxi) = tentative + heuristic([nyi, nxi], goalIdx, xVec, yVec);
        end
    end

    error('ObstacleGallery:NoPath', 'No path found for %s.', scenario.name);
end

function idx = nearestFreeIndex(point, xVec, yVec, free)
    [~, ix] = min(abs(xVec - point(1)));
    [~, iy] = min(abs(yVec - point(2)));
    if free(iy, ix)
        idx = [iy, ix];
        return;
    end

    [fy, fx] = find(free);
    d = hypot(xVec(fx) - point(1), yVec(fy) - point(2));
    [~, best] = min(d);
    idx = [fy(best), fx(best)];
end

function h = heuristic(a, b, xVec, yVec)
    h = hypot(xVec(a(2)) - xVec(b(2)), yVec(a(1)) - yVec(b(1)));
end

function path = reconstructPath(cameFrom, current, xVec, yVec)
    pathIdx = current;
    linearCurrent = sub2ind(size(cameFrom), current(1), current(2));
    while cameFrom(linearCurrent) ~= 0
        linearCurrent = cameFrom(linearCurrent);
        [iy, ix] = ind2sub(size(cameFrom), linearCurrent);
        pathIdx = [[iy, ix]; pathIdx]; %#ok<AGROW>
    end

    path = zeros(size(pathIdx, 1), 2);
    for i = 1:size(pathIdx, 1)
        path(i, :) = [xVec(pathIdx(i, 2)), yVec(pathIdx(i, 1))];
    end
end

function path = smoothPathSafe(rawPath, scenario)
    path = resamplePath(rawPath, scenario.pathSpacing);
    if size(path, 1) < 7
        return;
    end

    window = 5;
    candidate = path;
    candidate(:, 1) = smoothdata(path(:, 1), 'movmean', window);
    candidate(:, 2) = smoothdata(path(:, 2), 'movmean', window);
    candidate(1, :) = scenario.start;
    candidate(end, :) = scenario.goal;

    if pathIsFree(candidate, scenario)
        path = candidate;
    end
end

function path = resamplePath(pathIn, spacing)
    delta = diff(pathIn, 1, 1);
    segLen = hypot(delta(:, 1), delta(:, 2));
    s = [0; cumsum(segLen)];
    keep = [true; diff(s) > 1e-8];
    s = s(keep);
    pathIn = pathIn(keep, :);

    sample = (0:spacing:s(end))';
    if sample(end) < s(end)
        sample(end + 1) = s(end); %#ok<AGROW>
    end

    path = [interp1(s, pathIn(:, 1), sample, 'linear'), ...
            interp1(s, pathIn(:, 2), sample, 'linear')];
end

function ok = pathIsFree(path, scenario)
    ok = true;
    for i = 1:size(path, 1)
        if ~isFreePoint(path(i, :), scenario)
            ok = false;
            return;
        end
    end
end

%% Simulation
function result = simulateFollower(scenario, reference)
    pos = scenario.start;
    vel = [0, 0];
    yaw = atan2(reference(2, 2) - reference(1, 2), ...
                reference(2, 1) - reference(1, 1));
    idx = 2;

    trajectory = zeros(scenario.maxSteps, 2);
    yawHistory = zeros(scenario.maxSteps, 1);
    stateHistory = strings(scenario.maxSteps, 1);
    setpoints = zeros(scenario.maxSteps, 2);
    time = zeros(scenario.maxSteps, 1);

    for k = 1:scenario.maxSteps
        while idx < size(reference, 1) && norm(reference(idx, :) - pos) < scenario.lookahead
            idx = idx + 1;
        end

        target = reference(idx, :);
        err = target - pos;
        dist = norm(err);
        if dist > 1e-9
            dir = err / dist;
        else
            dir = [cos(yaw), sin(yaw)];
        end

        speed = min(scenario.speed, scenario.trackingGain * dist);
        velCmd = speed * dir;
        vel = 0.72 * vel + 0.28 * velCmd;
        posNext = pos + vel * scenario.dt;

        if ~isFreePoint(posNext, scenario)
            vel = 0.35 * vel;
            posNext = pos + 0.25 * (target - pos) * scenario.dt;
        end

        yaw = atan2(vel(2), vel(1));
        pos = posNext;

        trajectory(k, :) = pos;
        yawHistory(k) = yaw;
        setpoints(k, :) = target;
        time(k) = (k - 1) * scenario.dt;
        if idx >= size(reference, 1) - 2
            stateHistory(k) = "AT_GOAL";
        elseif distanceToNearestObstacle(pos, scenario) < 2 * scenario.safeDistance
            stateHistory(k) = "AVOIDING";
        else
            stateHistory(k) = "TRACKING";
        end

        if norm(pos - scenario.goal) < max(0.20, 0.6 * scenario.lookahead)
            trajectory = trajectory(1:k, :);
            yawHistory = yawHistory(1:k);
            setpoints = setpoints(1:k, :);
            stateHistory = stateHistory(1:k);
            time = time(1:k);
            break;
        end
    end

    result = struct();
    result.name = scenario.name;
    result.scenario = scenario;
    result.trajectory = trajectory;
    result.yawHistory = yawHistory;
    result.setpoints = setpoints;
    result.stateHistory = stateHistory;
    result.time = time;
    result.finalError = norm(trajectory(end, :) - scenario.goal);
    result.minClearance = minTrajectoryClearance(trajectory, scenario);
    result.collisionFree = trajectoryIsFree(trajectory, scenario);
end

%% Validation and metrics
function validateScenario(result)
    fprintf('  Samples:        %d\n', size(result.trajectory, 1));
    fprintf('  Final error:    %.3f m\n', result.finalError);
    fprintf('  Min clearance:  %.3f m\n', result.minClearance);
    fprintf('  Collision-free: %d\n', result.collisionFree);

    assert(result.finalError < max(0.35, result.scenario.lookahead), ...
        'ObstacleGallery:GoalMissed', '%s did not reach the goal.', result.name);
    assert(result.collisionFree, ...
        'ObstacleGallery:Collision', '%s collided with an obstacle/wall.', result.name);
    assert(result.minClearance >= -1e-6, ...
        'ObstacleGallery:NegativeClearance', '%s has negative clearance.', result.name);
end

function ok = trajectoryIsFree(trajectory, scenario)
    ok = true;
    for i = 1:size(trajectory, 1)
        if ~isFreePoint(trajectory(i, :), scenario)
            ok = false;
            return;
        end
    end
end

function minClearance = minTrajectoryClearance(trajectory, scenario)
    minClearance = inf;
    for i = 1:size(trajectory, 1)
        [~, clearance] = isFreePoint(trajectory(i, :), scenario);
        minClearance = min(minClearance, clearance);
    end
end

function [free, clearance] = isFreePoint(point, scenario)
    switch scenario.kind
        case 'circles'
            insideBounds = point(1) >= scenario.xLim(1) && point(1) <= scenario.xLim(2) && ...
                           point(2) >= scenario.yLim(1) && point(2) <= scenario.yLim(2);
            clearance = distanceToNearestObstacle(point, scenario) - scenario.vehicleRadius;
            free = insideBounds && clearance > 0;

        case 'corridor'
            poly = scenario.freePolygon;
            inside = inpolygon(point(1), point(2), poly(:, 1), poly(:, 2));
            clearance = distanceToPolygonBoundary(point, poly) - scenario.vehicleRadius;
            free = inside && clearance > 0;

        otherwise
            error('ObstacleGallery:BadScenarioKind', 'Unknown scenario kind.');
    end
end

function d = distanceToNearestObstacle(point, scenario)
    if isempty(scenario.obstacles)
        d = inf;
        return;
    end

    d = inf;
    for i = 1:size(scenario.obstacles, 1)
        obs = scenario.obstacles(i, :);
        d = min(d, norm(point - obs(1:2)) - obs(3));
    end
end

function d = distanceToPolygonBoundary(point, polygon)
    d = inf;
    n = size(polygon, 1);
    for i = 1:n
        a = polygon(i, :);
        b = polygon(mod(i, n) + 1, :);
        d = min(d, pointToSegmentDistance(point, a, b));
    end
end

function d = pointToSegmentDistance(p, a, b)
    ab = b - a;
    t = dot(p - a, ab) / max(dot(ab, ab), 1e-12);
    t = max(0, min(1, t));
    closest = a + t * ab;
    d = norm(p - closest);
end

%% Output
function saveScenarioResult(result, resultsDir)
    pngFile = fullfile(resultsDir, [result.name, '_gallery.png']);
    gifFile = fullfile(resultsDir, [result.name, '_animation.gif']);
    matFile = fullfile(resultsDir, [result.name, '_gallery_result.mat']);

    fig = plotScenario(result);
    saveas(fig, pngFile);
    animateScenario(result, gifFile);
    save(matFile, 'result');

    fprintf('  Saved PNG:       %s\n', pngFile);
    fprintf('  Saved GIF:       %s\n', gifFile);
    fprintf('  Saved MAT:       %s\n', matFile);
end

function fig = plotScenario(result)
    s = result.scenario;
    fig = figure('Name', result.name, 'Position', [100 100 760 640], 'Color', 'w');
    hold on; grid on; axis equal;

    drawScenarioGeometry(s);
    plot(result.trajectory(:, 1), result.trajectory(:, 2), '-', ...
        'Color', [0.00 0.55 0.55], 'LineWidth', 2.0, 'DisplayName', 'Tracked path');
    plot(result.setpoints(:, 1), result.setpoints(:, 2), ':', ...
        'Color', [0.10 0.30 0.90], 'LineWidth', 1.1, 'DisplayName', 'Planner setpoint');
    plot(result.reference(:, 1), result.reference(:, 2), '--', ...
        'Color', [0.95 0.35 0.10], 'LineWidth', 1.6, 'DisplayName', 'Reference');
    plot(s.start(1), s.start(2), 'o', 'Color', [0.0 0.45 0.0], ...
        'MarkerFaceColor', [0.0 0.75 0.0], 'MarkerSize', 7, 'DisplayName', 'Start');
    plot(s.goal(1), s.goal(2), 'o', 'Color', [0.05 0.15 0.75], ...
        'MarkerFaceColor', [0.15 0.40 1.0], 'MarkerSize', 7, 'DisplayName', 'End');
    drawDrone2D(result.trajectory(end, :), result.yawHistory(end), s.vehicleRadius, [0.0 0.55 0.55]);

    xlabel('x (m)');
    ylabel('y (m)');
    title(s.title);
    xlim(s.xLim);
    ylim(s.yLim);
    legend('Location', 'best');
end

function drawScenarioGeometry(s)
    switch s.kind
        case 'circles'
            th = linspace(0, 2*pi, 100);
            for i = 1:size(s.obstacles, 1)
                obs = s.obstacles(i, :);
                if i == 1
                    plot(obs(1) + (obs(3) + s.safeDistance) * cos(th), ...
                         obs(2) + (obs(3) + s.safeDistance) * sin(th), ...
                         ':', 'Color', [0.80 0.62 0.05], 'LineWidth', 1.1, ...
                         'DisplayName', 'Safe distance');
                    fill(obs(1) + obs(3) * cos(th), obs(2) + obs(3) * sin(th), ...
                         [0.90 0.90 0.90], 'EdgeColor', [0.10 0.10 0.10], ...
                         'FaceAlpha', 0.45, 'LineWidth', 1.0, ...
                         'DisplayName', 'Obstacle');
                else
                    plot(obs(1) + (obs(3) + s.safeDistance) * cos(th), ...
                         obs(2) + (obs(3) + s.safeDistance) * sin(th), ...
                         ':', 'Color', [0.80 0.62 0.05], 'LineWidth', 1.1, ...
                         'HandleVisibility', 'off');
                    fill(obs(1) + obs(3) * cos(th), obs(2) + obs(3) * sin(th), ...
                         [0.90 0.90 0.90], 'EdgeColor', [0.10 0.10 0.10], ...
                         'FaceAlpha', 0.45, 'LineWidth', 1.0, ...
                         'HandleVisibility', 'off');
                end
            end

        case 'corridor'
            p = s.freePolygon;
            patch(p(:, 1), p(:, 2), [0.97 0.97 0.97], ...
                'EdgeColor', [0.05 0.05 0.05], 'LineWidth', 2.0, ...
                'DisplayName', 'Free corridor');
            d = s.destination;
            x = [d(1), d(1) + d(3), d(1) + d(3), d(1)];
            y = [d(2), d(2), d(2) + d(4), d(2) + d(4)];
            patch(x, y, [1.0 0.20 0.25], 'FaceAlpha', 0.75, ...
                'EdgeColor', 'none', 'DisplayName', 'Destination');
    end
end

function animateScenario(result, gifFile)
    s = result.scenario;
    fig = figure('Name', [result.name, ' Animation'], ...
        'Position', [120 120 760 640], 'Color', 'w');

    frameIdx = unique(round(linspace(1, size(result.trajectory, 1), 90)));
    for f = 1:numel(frameIdx)
        k = frameIdx(f);
        clf(fig);
        hold on; grid on; axis equal;

        drawScenarioGeometry(s);
        plot(result.trajectory(1:k, 1), result.trajectory(1:k, 2), '-', ...
            'Color', [0.00 0.55 0.55], 'LineWidth', 2.2);
        plot(result.setpoints(max(1, k-25):k, 1), result.setpoints(max(1, k-25):k, 2), ...
            ':', 'Color', [0.10 0.30 0.90], 'LineWidth', 1.2);
        plot(result.reference(:, 1), result.reference(:, 2), '--', ...
            'Color', [0.95 0.35 0.10], 'LineWidth', 1.4);
        plot(s.start(1), s.start(2), 'o', 'Color', [0.0 0.45 0.0], ...
            'MarkerFaceColor', [0.0 0.75 0.0], 'MarkerSize', 7);
        plot(s.goal(1), s.goal(2), 'o', 'Color', [0.05 0.15 0.75], ...
            'MarkerFaceColor', [0.15 0.40 1.0], 'MarkerSize', 7);
        drawDrone2D(result.trajectory(k, :), result.yawHistory(k), ...
            s.vehicleRadius, [0.0 0.55 0.55]);

        xlabel('x (m)');
        ylabel('y (m)');
        title(sprintf('%s | t = %.1f s | %s', ...
            result.name, result.time(k), result.stateHistory(k)));
        xlim(s.xLim);
        ylim(s.yLim);
        drawnow;

        frameData = getframe(fig);
        [im, map] = rgb2ind(frame2im(frameData), 256);
        if f == 1
            imwrite(im, map, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', 0.07);
        else
            imwrite(im, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.07);
        end
    end

    close(fig);
end

function drawDrone2D(pos, yaw, radius, color)
    th = linspace(0, 2*pi, 40);
    fill(pos(1) + radius * cos(th), pos(2) + radius * sin(th), color, ...
        'FaceAlpha', 0.25, 'EdgeColor', color, 'LineWidth', 1.3, ...
        'HandleVisibility', 'off');
    nose = pos + 2.2 * radius * [cos(yaw), sin(yaw)];
    plot([pos(1), nose(1)], [pos(2), nose(2)], '-', ...
        'Color', color, 'LineWidth', 2, 'HandleVisibility', 'off');
    arm = radius * [-sin(yaw), cos(yaw)];
    plot([pos(1)-arm(1), pos(1)+arm(1)], [pos(2)-arm(2), pos(2)+arm(2)], ...
        '-', 'Color', color, 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
