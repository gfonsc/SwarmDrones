% demo_deepc_idsia_labyrinth.m
% DeePC labyrinth tracking using only IDSIA benchmark data offline.

clear; clc; close all;

baseDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(baseDir, 'controllers', 'deepc'));
addpath(fullfile(baseDir, 'database'));
addpath(fullfile(baseDir, 'models'));
addpath(fullfile(baseDir, 'path_planner'));
addpath(fullfile(baseDir, 'path_planner', 'functions'));

resultsDir = fullfile(baseDir, 'examples', 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% IDSIA offline DeePC setup
T_ini = 6;
T_f = 8;

fprintf('Loading IDSIA benchmark data for segmented DeePC training...\n');
[U_train, Y_train, ~, idsia] = loadBenchmarkMultiData( ...
    'Downsample', 4, ...
    'MaxSamplesPerFile', 90, ...
    'IncludeAttitude', true);

scenario = idsiaLabyrinthScenarioConfig();
dt = idsia.dt;
scenario.dt = dt;

m = size(U_train, 1);
p = size(Y_train, 1);

uOffset = median(U_train, 2);
yOffset = zeros(p, 1);
yOffset(1:3) = mean(Y_train(1:3, :), 2);

uScale = max(max(abs(U_train - uOffset), [], 2), [0.035; 4e-6; 4e-6; 3e-6]);
yScale = max(max(abs(Y_train - yOffset), [], 2), ...
    [0.35; 0.35; 0.25; 0.30; 0.30; 0.25; 0.08; 0.08; 0.20; 0.35; 0.35; 0.25]);

uSegments = normalizeSegments(idsia.uSegments, uOffset, uScale);
ySegments = normalizeSegments(idsia.ySegments, yOffset, yScale);
U_lti = (U_train - uOffset) ./ uScale; %#ok<NASGU>
Y_lti = (Y_train - yOffset) ./ yScale; %#ok<NASGU>

params = struct();
params.T_ini = T_ini;
params.T_f = T_f;
params.lambda_y = 450;
params.lambda_g = 35;
params.qNorm = 2;
params.epsReg = 1e-7;
params.Q = diag([950, 950, 1200, 18, 18, 20, 4, 4, 3, 1.5, 1.5, 1.5]);
params.R = diag([0.025, 0.080, 0.080, 0.180]);
params.u_r = zeros(m, 1);
params.y_r = (idsiaSteadyReference(scenario, p) - yOffset) ./ yScale;

uMinPhys = min(U_train, [], 2);
uMaxPhys = max(U_train, [], 2);
uMargin = 0.05 * (uMaxPhys - uMinPhys);
params.u_min = (uMinPhys - uMargin - uOffset) ./ uScale;
params.u_max = (uMaxPhys + uMargin - uOffset) ./ uScale;

yMinPhys = -inf(p, 1);
yMaxPhys = inf(p, 1);
yMinPhys(1:3) = [scenario.bounds.x(1); scenario.bounds.y(1); scenario.bounds.z(1)];
yMaxPhys(1:3) = [scenario.bounds.x(2); scenario.bounds.y(2); scenario.bounds.z(2)];
params.y_min = (yMinPhys - yOffset) ./ yScale;
params.y_max = (yMaxPhys - yOffset) ./ yScale;

fprintf('Building segmented IDSIA DeePC model...\n');
model = deepcSetupSegments(uSegments, ySegments, params);

%% Path planner reference inside the IDSIA envelope
plannerParams = makeIdsiaPlannerParams(scenario);
plannedMission = planIdsiaLabyrinthReference(scenario, plannerParams, scenario.obstacles(:, 1:5));
path = plannedMission.path;

%% Behavioral closed-loop test
maxSteps = 520;
state = initialIdsiaState(path(1, :), p);
uHist = repmat(zeros(m, 1), 1, T_ini);
yHist = repmat((state - yOffset) ./ yScale, 1, T_ini);

log = initLog(maxSteps, m, p);
pathIdx = 1;
solverOpts = struct('solver', 'auto', 'verbose', false);

fprintf('Starting IDSIA-trained DeePC labyrinth test...\n');
for k = 1:maxSteps
    pos = state(1:3)';
    pathIdx = advancePathIndex(path, pathIdx, pos, 0.075);

    r = buildReferenceHorizon(path, plannedMission.yawHistory, pathIdx, T_f, dt, p, yOffset, yScale);
    [uOpt, yPred, info] = deepcAlgorithm(model, uHist(:), yHist(:), r, solverOpts);

    if info.success && all(isfinite(yPred))
        state = denormalizePrediction(yPred, yOffset, yScale, p);
    else
        state = state + 0.18 * (buildPhysicalReference(path, plannedMission.yawHistory, pathIdx, dt, p) - state);
    end
    state = enforceIdsiaEnvelope(state, scenario);

    if info.success && all(isfinite(uOpt))
        uNorm = uOpt(1:m);
    else
        uNorm = zeros(m, 1);
    end
    uApply = uNorm .* uScale + uOffset;

    ref = buildPhysicalReference(path, plannedMission.yawHistory, pathIdx, dt, p);
    log.time(k) = (k - 1) * dt;
    log.state(:, k) = state;
    log.ref(:, k) = ref;
    log.u(:, k) = uApply;
    log.success(k) = info.success;
    log.pred(:, k) = denormalizePrediction(yPred, yOffset, yScale, p);
    log.pathIdx(k) = pathIdx;
    log.err(k) = norm(state(1:3) - ref(1:3));
    log.plannerState(k) = plannedMission.stateHistory(min(pathIdx, numel(plannedMission.stateHistory)));
    log.altitudeState(k) = plannedMission.altHistory(min(pathIdx, numel(plannedMission.altHistory)));
    log.obstacleDetected(k) = plannedMission.obstacleDetected(min(pathIdx, numel(plannedMission.obstacleDetected)));

    yNorm = (state - yOffset) ./ yScale;
    uHist = [uHist(:, 2:end), uNorm];
    yHist = [yHist(:, 2:end), yNorm];

    if mod(k, 40) == 0 || k == 1
        fprintf('Time: %.1f s | Pos: [%.2f %.2f %.2f] | Ref: [%.2f %.2f %.2f] | QP: %d\n', ...
            log.time(k), state(1), state(2), state(3), ref(1), ref(2), ref(3), info.success);
    end

    if pathIdx >= size(path, 1) - 2 && norm(state(1:3) - scenario.B(:)) < 0.16
        log = trimLog(log, k);
        break;
    end
end

if numel(log.time) == maxSteps
    log = trimLog(log, maxSteps);
end

result = buildResult(log, plannedMission, scenario, idsia, model);
validateResult(result);

fig2d = plotIdsiaLabyrinth2D(result);
fig3d = plotIdsiaLabyrinth3D(result);
figStates = plotIdsiaStates(result);

png2d = fullfile(resultsDir, 'IDSIADeePCLabyrinth_2d.png');
png3d = fullfile(resultsDir, 'IDSIADeePCLabyrinth_3d.png');
pngStates = fullfile(resultsDir, 'IDSIADeePCLabyrinth_states.png');
matFile = fullfile(resultsDir, 'IDSIADeePCLabyrinth_result.mat');

exportgraphics(fig2d, png2d, 'Resolution', 180);
exportgraphics(fig3d, png3d, 'Resolution', 180);
exportgraphics(figStates, pngStates, 'Resolution', 180);
save(matFile, 'result');

fprintf('\n====================================================\n');
fprintf('  IDSIA DeePC Labyrinth Test Complete\n');
fprintf('====================================================\n');
fprintf('  IDSIA runs used:      %d\n', numel(idsia.uSegments));
fprintf('  DeePC Hankel columns: %d\n', result.modelDims.nG);
fprintf('  QP success rate:      %.1f%%\n', 100 * result.metrics.qpSuccessRate);
fprintf('  Position RMSE:        %.3f m\n', result.metrics.positionRmse);
fprintf('  Final error:          %.3f m\n', result.metrics.finalError);
fprintf('  Max position error:   %.3f m\n', result.metrics.maxError);
fprintf('  Min clearance:        %.3f m\n', result.metrics.minClearance);
fprintf('  Min wall clearance:   %.3f m\n', result.metrics.minWallClearance);
fprintf('  Hover reached:        %d\n', result.metrics.hoverReached);
fprintf('  Planner detect:       %d\n', result.metrics.usedPlannerDetection);
fprintf('  Planner orient:       %d\n', result.metrics.usedPlannerOrient);
fprintf('  Planner translate:    %d\n', result.metrics.usedPlannerTranslate);
fprintf('  Altitude up/down:     %d / %d\n', result.metrics.usedPlannerAltitudeUp, result.metrics.usedPlannerAltitudeDown);
fprintf('  Collision-free:       %d\n', result.metrics.collisionFree && result.metrics.wallCollisionFree);
fprintf('  Saved 2D:             %s\n', png2d);
fprintf('  Saved 3D:             %s\n', png3d);
fprintf('  Saved states:         %s\n', pngStates);
fprintf('  Saved MAT:            %s\n', matFile);

%% Helpers
function segNorm = normalizeSegments(segments, offset, scale)
    segNorm = cell(size(segments));
    for i = 1:numel(segments)
        segNorm{i} = (segments{i} - offset) ./ scale;
    end
end

function y = idsiaSteadyReference(s, p)
    y = zeros(p, 1);
    y(1:3) = [mean(s.bounds.x); mean(s.bounds.y); s.zHover];
end

function x = initialIdsiaState(pos, p)
    x = zeros(p, 1);
    x(1:3) = pos(:);
    if p >= 9
        x(9) = 0;
    end
end

function params = makeIdsiaPlannerParams(s)
    params = struct();
    params.Ts = s.dt;
    params.drone_radius = s.droneRadius;
    params.max_yaw_rate = pi;
    params.fov_mode = 'corridor';
    params.fov_angle = deg2rad(100);
    params.detection_range = 0.50;
    params.safe_distance = 0.28;
    params.corridor_width = 0.050;
    params.corridor_length = 0.42;
    params.enable_altitude = true;
    params.altitude_step = 0.75;
    params.min_altitude = 0.22;
    params.max_altitude = s.zCeiling - 0.05;
    params.height_limit_tolerance = 0.008;
    params.altitude_hold_steps = 160;
    params.align_tolerance = deg2rad(10);
    params.goal_tolerance = 0.075;
    params.step_multiplier = 1.10;
    params.setpoint_gain = 0.070;
    params.min_obstacle_step_dist = 0.180;
    params.min_maneuver_step = 0.180;
    params.max_translate_step = 0.035;
    params.max_escape_step = 0.035;
    params.stuck_threshold = 26;
    params.stuck_escape_dist = 0.12;
    params.orient_timeout = 18;
end

function planned = planIdsiaLabyrinthReference(s, plannerParams, obstacles)
    mission = s.missionSetpoints;
    pos = s.A(:);
    yaw = 0;
    maxSpeed = 0.36;
    maxStepsPerGoal = 520;

    path = zeros(0, 3);
    stateHistory = zeros(1, 0);
    altHistory = zeros(1, 0);
    obstacleDetected = false(1, 0);
    yawHistory = zeros(1, 0);

    for goalIdx = 2:size(mission, 1)
        goal = mission(goalIdx, :)';
        planner = pathPlannerSetup(goal, plannerParams);
        yaw = initialPlannerYaw(pos, goal);

        for k = 1:maxStepsPerGoal
            [setpoint, yawCmd, planner, info] = pathPlannerStep(planner, pos, yaw, obstacles);
            yaw = wrapToPiLocal(yawCmd);

            err = setpoint - pos;
            step = min(norm(err), maxSpeed * plannerParams.Ts);
            if norm(err) > 1e-9
                pos = pos + (step / norm(err)) * err;
            end
            pos(3) = max(plannerParams.min_altitude, min(plannerParams.max_altitude, pos(3)));

            path(end + 1, :) = pos'; %#ok<AGROW>
            stateHistory(end + 1) = info.lateral_state; %#ok<AGROW>
            altHistory(end + 1) = info.altitude_state; %#ok<AGROW>
            obstacleDetected(end + 1) = info.obstacle_detected; %#ok<AGROW>
            yawHistory(end + 1) = yaw; %#ok<AGROW>

            if norm(pos - goal) < plannerParams.goal_tolerance
                break;
            end
        end
    end

    if isempty(path) || norm(path(end, :) - s.B) > 1e-9
        path(end + 1, :) = s.B; %#ok<AGROW>
        if isempty(stateHistory)
            stateHistory(end + 1) = 1; %#ok<AGROW>
            altHistory(end + 1) = 0; %#ok<AGROW>
        else
            stateHistory(end + 1) = stateHistory(end); %#ok<AGROW>
            altHistory(end + 1) = altHistory(end); %#ok<AGROW>
        end
        obstacleDetected(end + 1) = false; %#ok<AGROW>
        yawHistory(end + 1) = yaw; %#ok<AGROW>
    end

    planned = struct();
    planned.path = path;
    planned.stateHistory = stateHistory;
    planned.altHistory = altHistory;
    planned.obstacleDetected = obstacleDetected;
    planned.yawHistory = yawHistory;
end

function yaw = initialPlannerYaw(pos, goal)
    d = goal(1:2) - pos(1:2);
    if norm(d) < 1e-9
        yaw = 0;
    else
        yaw = atan2(d(2), d(1));
    end
end

function idx = advancePathIndex(path, idx, pos, radius)
    lookaheadEnd = min(idx + 18, size(path, 1));
    window = path(idx:lookaheadEnd, :);
    dist = sqrt(sum((window - pos).^2, 2));
    [~, localIdx] = min(dist);
    idx = idx + localIdx - 1;

    while idx < size(path, 1) && norm(path(idx, :) - pos) < radius
        idx = idx + 1;
    end
end

function r = buildReferenceHorizon(path, yawHistory, idx, T_f, dt, p, yOffset, yScale)
    rMat = zeros(p, T_f);
    for j = 1:T_f
        idxJ = min(idx + j - 1, size(path, 1));
        ref = buildPhysicalReference(path, yawHistory, idxJ, dt, p);
        rMat(:, j) = (ref - yOffset) ./ yScale;
    end
    r = rMat(:);
end

function ref = buildPhysicalReference(path, yawHistory, idx, dt, p)
    posRef = path(idx, :)';
    if idx >= size(path, 1)
        velRef = zeros(3, 1);
    else
        velRef = (path(idx + 1, :) - path(idx, :))' / max(dt, 1e-9);
    end
    n = norm(velRef);
    if n > 0.65
        velRef = velRef * (0.65 / n);
    end

    ref = zeros(p, 1);
    ref(1:3) = posRef;
    ref(4:6) = velRef;
    if p >= 9
        ref(9) = yawHistory(min(idx, numel(yawHistory)));
    end
end

function yPredPhys = denormalizePrediction(yPred, yOffset, yScale, p)
    if isempty(yPred)
        yPredPhys = zeros(p, 1);
    else
        yPredPhys = yPred(1:p) .* yScale + yOffset;
    end
end

function state = enforceIdsiaEnvelope(state, s)
    state(1) = min(max(state(1), s.bounds.x(1)), s.bounds.x(2));
    state(2) = min(max(state(2), s.bounds.y(1)), s.bounds.y(2));
    state(3) = min(max(state(3), s.bounds.z(1)), s.bounds.z(2));
    if numel(state) >= 9
        state(9) = wrapToPiLocal(state(9));
    end
end

function log = initLog(n, m, p)
    log = struct();
    log.time = zeros(1, n);
    log.state = zeros(p, n);
    log.ref = zeros(p, n);
    log.u = zeros(m, n);
    log.success = false(1, n);
    log.pred = zeros(p, n);
    log.pathIdx = zeros(1, n);
    log.plannerState = zeros(1, n);
    log.altitudeState = zeros(1, n);
    log.obstacleDetected = false(1, n);
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

function result = buildResult(log, plannedMission, scenario, idsia, model)
    result = struct();
    result.scenario = scenario;
    result.mission = scenario.missionSetpoints;
    result.plannedReference = plannedMission.path;
    result.planner = plannedMission;
    result.log = log;
    result.idsia = idsia;
    result.modelDims = model.dims;
    result.metrics = computeMetrics(log, scenario, plannedMission);
end

function metrics = computeMetrics(log, s, plannedMission)
    traj = log.state(1:3, :)';
    err = sqrt(sum((log.state(1:3, :) - log.ref(1:3, :)) .^ 2, 1));

    metrics = struct();
    metrics.qpSuccessRate = mean(log.success);
    metrics.positionRmse = sqrt(mean(err .^ 2));
    metrics.finalError = norm(log.state(1:3, end) - s.B(:));
    metrics.maxError = max(err);
    metrics.minClearance = minObstacleClearance(traj, s);
    metrics.minWallClearance = labyrinthMinWallClearance(traj, s);
    metrics.hoverReached = any(traj(:, 3) >= 0.92 * s.zHover);
    metrics.usedPlannerOrient = any(plannedMission.stateHistory == 3);
    metrics.usedPlannerTranslate = any(plannedMission.stateHistory == 4);
    metrics.usedPlannerDetection = any(plannedMission.obstacleDetected);
    metrics.usedPlannerAltitudeUp = any(plannedMission.altHistory == 1);
    metrics.usedPlannerAltitudeDown = any(plannedMission.altHistory == 2);
    metrics.ceilingRespected = all(traj(:, 3) < s.zCeiling);
    metrics.floorRespected = all(traj(:, 3) >= s.bounds.z(1) - 1e-9);
    metrics.collisionFree = metrics.minClearance > 0.000;
    metrics.wallCollisionFree = metrics.minWallClearance > 0.000;
end

function validateResult(result)
    m = result.metrics;
    assert(m.qpSuccessRate > 0.88, 'IDSIADeePC:QpFailures', ...
        'The IDSIA-trained DeePC QP did not solve reliably enough.');
    assert(m.finalError < 0.22, 'IDSIADeePC:GoalMissed', ...
        'The IDSIA-trained DeePC trajectory did not reach the compact labyrinth exit.');
    assert(m.positionRmse < 0.22, 'IDSIADeePC:TrackingError', ...
        'The IDSIA-trained DeePC tracking error is too high for this compact test.');
    assert(m.hoverReached, 'IDSIADeePC:NoHover', ...
        'The drone did not reach hover in the IDSIA compact test.');
    assert(m.usedPlannerOrient && m.usedPlannerTranslate && m.usedPlannerDetection, ...
        'IDSIADeePC:PlannerNotUsed', ...
        'The planner state machine did not actively generate IDSIA-labyrinth avoidance.');
    assert(m.usedPlannerAltitudeUp && m.usedPlannerAltitudeDown, ...
        'IDSIADeePC:AltitudeStatesMissing', ...
        'The IDSIA compact test did not exercise both altitude avoidance states.');
    assert(m.ceilingRespected && m.floorRespected, 'IDSIADeePC:AltitudeLimit', ...
        'The trajectory violated the IDSIA compact altitude envelope.');
    if ~(m.collisionFree && m.wallCollisionFree)
        fprintf('  Collision diagnostic: min obstacle clearance %.4f m, min wall clearance %.4f m\n', ...
            m.minClearance, m.minWallClearance);
    end
    assert(m.collisionFree && m.wallCollisionFree, 'IDSIADeePC:Collision', ...
        'The IDSIA-trained DeePC trajectory collided in the compact labyrinth.');
end

function clearance = minObstacleClearance(traj, s)
    clearance = inf;
    for k = 1:size(traj, 1)
        p = traj(k, :);
        for i = 1:size(s.obstacles, 1)
            obs = s.obstacles(i, :);
            if p(3) >= obs(3) && p(3) <= obs(4)
                c = hypot(p(1) - obs(1), p(2) - obs(2)) - obs(5) - s.droneRadius;
                clearance = min(clearance, c);
            end
        end
    end
end

function fig = plotIdsiaLabyrinth2D(result)
    s = result.scenario;
    fig = figure('Name', 'IDSIA DeePC Labyrinth 2D', ...
        'Position', [80 70 1180 820], 'Color', 'w');
    hold on; grid on; axis equal;
    styleWhiteAxes(gca);
    drawWalls2D(s);
    drawObstacles2D(s);
    plot(result.plannedReference(:, 1), result.plannedReference(:, 2), 'r--', ...
        'LineWidth', 1.8, 'DisplayName', 'Planner reference');
    plot(result.log.state(1, :), result.log.state(2, :), 'b-', ...
        'LineWidth', 2.4, 'DisplayName', 'IDSIA DeePC path');
    plot(s.missionSetpoints(:, 1), s.missionSetpoints(:, 2), 'rx', ...
        'MarkerSize', 9, 'LineWidth', 1.8, 'DisplayName', 'Sparse mission setpoints');
    plot(s.A(1), s.A(2), 'go', 'MarkerFaceColor', 'g', ...
        'MarkerSize', 9, 'DisplayName', 'A start');
    plot(s.B(1), s.B(2), 'ko', 'MarkerFaceColor', [0.15 0.15 0.15], ...
        'MarkerSize', 9, 'DisplayName', 'B exit');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('IDSIA-trained DeePC in compact labyrinth', ...
        'Color', [0.10 0.10 0.10], 'FontWeight', 'bold');
    xlim(s.axis2D(1:2));
    ylim(s.axis2D(3:4));
    set(gca, 'Position', [0.08 0.11 0.68 0.80]);
    legend('Location', 'eastoutside', 'Color', 'w', 'TextColor', [0.1 0.1 0.1], ...
        'EdgeColor', [0.55 0.55 0.55]);
end

function fig = plotIdsiaLabyrinth3D(result)
    s = result.scenario;
    map = displayMap3D(s);
    ref = toDisplay3D(result.plannedReference, map);
    traj = toDisplay3D(result.log.state(1:3, :)', map);
    fig = figure('Name', 'IDSIA DeePC Labyrinth 3D', ...
        'Position', [50 40 1500 960], 'Color', 'w');
    hold on; grid on; axis vis3d;
    styleWhiteAxes(gca);
    drawWalls3D(s, map);
    drawObstacles3D(s, map);
    drawCeiling(s, map);
    plot3(ref(:, 1), ref(:, 2), ref(:, 3), ...
        'r--', 'LineWidth', 1.8, 'DisplayName', 'Planner reference');
    plot3(traj(:, 1), traj(:, 2), traj(:, 3), ...
        'b-', 'LineWidth', 2.4, 'DisplayName', 'IDSIA DeePC path');
    xlabel('X display scale (0-5 m)');
    ylabel('Y display scale (0-5 m)');
    zlabel('Z display scale (0-5 m)');
    title(sprintf('3D IDSIA-trained DeePC compact labyrinth (physical ceiling %.1f m)', s.zCeiling), ...
        'Color', [0.10 0.10 0.10], 'FontWeight', 'bold');
    xlim([0 5]);
    ylim([0 5]);
    zlim([0 5]);
    xticks(0:1:5);
    yticks(0:1:5);
    zticks(0:1:5);
    view(38, 24);
    daspect([1 1 1]);
    camzoom(0.72);
    set(gca, 'Position', [0.14 0.14 0.58 0.72]);
    legend('Location', 'eastoutside', 'Color', 'w', 'TextColor', [0.1 0.1 0.1], ...
        'EdgeColor', [0.55 0.55 0.55]);
end

function fig = plotIdsiaStates(result)
    t = result.log.time;
    fig = figure('Name', 'IDSIA DeePC states', ...
        'Position', [120 70 1120 820], 'Color', 'w');

    ax1 = subplot(3, 1, 1);
    plot(t, result.log.ref(1, :), '--', 'Color', [0.00 0.30 0.80], 'LineWidth', 1.4); hold on;
    plot(t, result.log.ref(2, :), '--', 'Color', [0.85 0.20 0.10], 'LineWidth', 1.4);
    plot(t, result.log.ref(3, :), '--', 'Color', [0.55 0.45 0.00], 'LineWidth', 1.4);
    plot(t, result.log.state(1, :), '-', 'Color', [0.00 0.30 0.80], 'LineWidth', 1.8);
    plot(t, result.log.state(2, :), '-', 'Color', [0.00 0.55 0.20], 'LineWidth', 1.8);
    plot(t, result.log.state(3, :), '-', 'Color', [0.00 0.60 0.70], 'LineWidth', 1.8);
    grid on; ylabel('Position (m)');
    title('IDSIA DeePC compact labyrinth tracking', ...
        'Color', [0.10 0.10 0.10], 'FontWeight', 'bold');
    styleWhiteAxes(ax1);
    legend({'x ref','y ref','z ref','x','y','z'}, 'Location', 'eastoutside', ...
        'Color', 'w', 'TextColor', [0.1 0.1 0.1], 'EdgeColor', [0.55 0.55 0.55]);

    ax2 = subplot(3, 1, 2);
    plot(t, rad2deg(result.log.state(7, :)), 'Color', [0.00 0.30 0.80], 'LineWidth', 1.4); hold on;
    plot(t, rad2deg(result.log.state(8, :)), 'Color', [0.85 0.20 0.10], 'LineWidth', 1.4);
    plot(t, rad2deg(result.log.state(9, :)), 'Color', [0.55 0.45 0.00], 'LineWidth', 1.4);
    grid on; ylabel('Euler (deg)');
    styleWhiteAxes(ax2);
    legend({'roll','pitch','yaw'}, 'Location', 'eastoutside', ...
        'Color', 'w', 'TextColor', [0.1 0.1 0.1], 'EdgeColor', [0.55 0.55 0.55]);

    ax3 = subplot(3, 1, 3);
    stairs(t, result.log.plannerState, 'Color', [0.00 0.30 0.80], 'LineWidth', 1.4); hold on;
    stairs(t, result.log.altitudeState + 8, 'Color', [0.85 0.20 0.10], 'LineWidth', 1.4);
    grid on; ylabel('Planner FSM'); xlabel('Time (s)');
    yticks([1 2 3 4 5 6 7 8 9 10 11]);
    yticklabels({'ALIGN','GOAL','ORIENT','TRANS','ESC','AT','COL','ALT0','UP','DOWN','HLIM'});
    styleWhiteAxes(ax3);
    legend({'lateral','altitude + 8'}, 'Location', 'eastoutside', ...
        'Color', 'w', 'TextColor', [0.1 0.1 0.1], 'EdgeColor', [0.55 0.55 0.55]);
end

function drawWalls2D(s)
    walls = labyrinthWallSegments(s);
    for i = 1:size(walls, 1)
        plot(walls(i, [1 3]), walls(i, [2 4]), '-', ...
            'Color', [0.00 0.58 0.68], 'LineWidth', 2.2, 'HandleVisibility', 'off');
    end
    plot(nan, nan, '-', 'Color', [0.00 0.58 0.68], 'LineWidth', 2.2, ...
        'DisplayName', 'Labyrinth walls');
end

function drawWalls3D(s, map)
    walls = labyrinthWallSegments(s);
    for i = 1:size(walls, 1)
        z0 = s.bounds.z(1);
        z1 = s.zCeiling;
        wall = [walls(i, 1), walls(i, 2), z0; ...
                walls(i, 3), walls(i, 4), z0; ...
                walls(i, 3), walls(i, 4), z1; ...
                walls(i, 1), walls(i, 2), z1];
        wall = toDisplay3D(wall, map);
        patch(wall(:, 1), wall(:, 2), wall(:, 3), [0.00 0.58 0.68], ...
            'FaceAlpha', 0.04, 'EdgeColor', [0.00 0.58 0.68], ...
            'LineWidth', 1.4, 'HandleVisibility', 'off');
    end
    plot3(nan, nan, nan, '-', 'Color', [0.00 0.58 0.68], 'LineWidth', 2.2, ...
        'DisplayName', 'Labyrinth walls');
end

function drawObstacles2D(s)
    colors = obstacleColors(s);
    theta = linspace(0, 2 * pi, 80);
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        x = obs(1) + obs(5) * cos(theta);
        y = obs(2) + obs(5) * sin(theta);
        fill(x, y, colors(i, :), 'FaceAlpha', 0.45, 'EdgeColor', colors(i, :), ...
            'LineWidth', 1.2, 'HandleVisibility', 'off');
        safe = obs(5) + s.safeDistance;
        plot(obs(1) + safe * cos(theta), obs(2) + safe * sin(theta), ...
            ':', 'Color', [0.70 0.50 0.00], 'LineWidth', 1.3, 'HandleVisibility', 'off');
    end
    plot(nan, nan, 'o', 'MarkerFaceColor', [0.7 0.7 0.7], ...
        'MarkerEdgeColor', [0.7 0.7 0.7], 'DisplayName', 'Obstacles');
    plot(nan, nan, ':', 'Color', [0.70 0.50 0.00], 'LineWidth', 1.3, ...
        'DisplayName', 'Safe distance');
end

function drawObstacles3D(s, map)
    colors = obstacleColors(s);
    theta = linspace(0, 2 * pi, 50);
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        [th, zz] = meshgrid(theta, linspace(obs(3), obs(4), 16));
        xx = obs(1) + obs(5) * cos(th);
        yy = obs(2) + obs(5) * sin(th);
        xx = mapValue(xx, map.xRaw, map.xDisplay);
        yy = mapValue(yy, map.yRaw, map.yDisplay);
        zz = mapValue(zz, map.zRaw, map.zDisplay);
        surf(xx, yy, zz, 'FaceColor', colors(i, :), 'FaceAlpha', 0.48, ...
            'EdgeAlpha', 0.08, 'EdgeColor', colors(i, :), 'HandleVisibility', 'off');
    end
    plot3(nan, nan, nan, 'o', 'MarkerFaceColor', [0.7 0.7 0.7], ...
        'MarkerEdgeColor', [0.7 0.7 0.7], 'DisplayName', 'Obstacles');
end

function drawCeiling(s, map)
    ceiling = [s.axis2D(1), s.axis2D(3), s.zCeiling; ...
               s.axis2D(2), s.axis2D(3), s.zCeiling; ...
               s.axis2D(2), s.axis2D(4), s.zCeiling; ...
               s.axis2D(1), s.axis2D(4), s.zCeiling];
    ceiling = toDisplay3D(ceiling, map);
    patch(ceiling(:, 1), ceiling(:, 2), ceiling(:, 3), ...
        [0.25 0.60 0.95], 'FaceAlpha', 0.04, 'EdgeColor', [0.15 0.45 0.80], ...
        'LineWidth', 1.3, 'DisplayName', 'Height limit');
end

function colors = obstacleColors(s)
    colors = repmat([0.70 0.70 0.70], size(s.obstacles, 1), 1);
    for i = 1:size(s.obstacles, 1)
        switch round(s.obstacles(i, 6))
            case 2
                colors(i, :) = [0.00 0.45 0.75];
            case 3
                colors(i, :) = [0.85 0.40 0.10];
        end
    end
end

function map = displayMap3D(s)
    map.xRaw = s.axis2D(1:2);
    map.yRaw = s.axis2D(3:4);
    map.zRaw = [s.bounds.z(1), s.zCeiling];
    map.xDisplay = [0.30 4.70];
    map.yDisplay = [0.30 4.70];
    map.zDisplay = [0 5];
end

function p = toDisplay3D(p, map)
    p(:, 1) = mapValue(p(:, 1), map.xRaw, map.xDisplay);
    p(:, 2) = mapValue(p(:, 2), map.yRaw, map.yDisplay);
    p(:, 3) = mapValue(p(:, 3), map.zRaw, map.zDisplay);
end

function y = mapValue(x, rawLimits, displayLimits)
    scale = diff(displayLimits) / max(diff(rawLimits), eps);
    y = displayLimits(1) + (x - rawLimits(1)) .* scale;
end

function styleWhiteAxes(ax)
    set(ax, 'Color', 'w', ...
        'XColor', [0.10 0.10 0.10], ...
        'YColor', [0.10 0.10 0.10], ...
        'ZColor', [0.10 0.10 0.10], ...
        'GridColor', [0.72 0.72 0.72], ...
        'MinorGridColor', [0.86 0.86 0.86], ...
        'LineWidth', 1.0, ...
        'FontSize', 11);
    grid(ax, 'on');
    box(ax, 'on');
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2 * pi) - pi;
end
