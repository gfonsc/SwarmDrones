% demo_deepc_labyrinth_dynamic_database.m
% Full 6-DOF dynamic DeePC test in the labyrinth envelope.

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

%% Offline dynamic database and DeePC setup
T_ini = 6;
T_f = 12;

fprintf('Loading dynamic labyrinth DeePC database...\n');
[U_train, Y_train, ~, meta] = loadLabyrinthDynamicData('Regenerate', true, ...
    'NumSamples', 620, 'Seed', 23);
scenario = meta.scenario;
dyn = scenario.dynamic;

m = size(U_train, 1);
p = size(Y_train, 1);

uOffset = [dyn.hoverThrust; 0; 0; 0];
yOffset = zeros(p, 1);
yOffset(1:3) = mean(Y_train(1:3, :), 2);

uScale = max(max(abs(U_train - uOffset), [], 2), [0.04; 5e-6; 5e-6; 4e-6]);
yScale = max(max(abs(Y_train - yOffset), [], 2), ...
    [1.00; 1.00; 0.50; 0.50; 0.50; 0.30; 0.08; 0.08; 0.50; 0.40; 0.40; 0.30]);

U_lti = (U_train - uOffset) ./ uScale;
Y_lti = (Y_train - yOffset) ./ yScale;

params = struct();
params.T_ini = T_ini;
params.T_f = T_f;
params.lambda_y = 600;
params.lambda_g = 20;
params.qNorm = 2;
params.epsReg = 1e-7;
params.Q = diag([620, 620, 760, 15, 15, 18, 8, 8, 45, 2, 2, 8]);
params.R = diag([0.020, 0.050, 0.050, 0.200]);
params.u_r = (uOffset - uOffset) ./ uScale;
params.y_r = (zeros(p, 1) - yOffset) ./ yScale;
params.u_min = (dyn.uMin - uOffset) ./ uScale;
params.u_max = (dyn.uMax - uOffset) ./ uScale;
params.y_min = (dyn.stateMin - yOffset) ./ yScale;
params.y_max = (dyn.stateMax - yOffset) ./ yScale;

fprintf('Building DeePC model for the dynamic labyrinth envelope...\n');
model = deepcSetup(U_lti, Y_lti, params);

%% Closed-loop dynamic mission
% The default plant mode mirrors demo_deepc_square: the DeePC behavioral
% model supplies the next measured dynamic state. Inputs are still physical
% thrust/torques learned from the 6-DOF dynamic database.
plantMode = 'behavioral'; % 'behavioral' or 'nonlinear6dof'
mission = scenario.missionSetpoints;
plannerObstacles = scenario.obstacles(:, 1:5);
plannerParams = makeLabyrinthPlannerParams(scenario, dyn);
plannedMission = planLabyrinthMissionReference(scenario, plannerParams, plannerObstacles);
path = plannedMission.path;
maxSteps = 950;
dt = dyn.dt;

if isfield(meta, 'X')
    state = meta.X(:, T_ini);
else
    state = Y_train(:, T_ini);
end
state(9) = wrapToPiLocal(state(9));

uHist = U_lti(:, 1:T_ini);
yHist = Y_lti(:, 1:T_ini);

log = initLog(maxSteps, m, p);
pathIdx = 1;
solverOpts = struct('solver', 'auto', 'verbose', false);

fprintf('Starting dynamic DeePC closed-loop simulation...\n');
for k = 1:maxSteps
    pos = state(1:3)';
    pathIdx = advancePathIndex(path, pathIdx, pos, 0.30);

    r = buildReferenceHorizon(path, pathIdx, T_f, dt, p, yOffset, yScale);
    [uOpt, yPred, info] = deepcAlgorithm(model, uHist(:), yHist(:), r, solverOpts);

    if info.success && all(isfinite(uOpt))
        uNorm = uOpt(1:m);
    else
        uNorm = zeros(m, 1);
    end

    uApply = uNorm .* uScale + uOffset;
    uApply = max(dyn.uMin, min(dyn.uMax, uApply));

    if strcmpi(plantMode, 'nonlinear6dof')
        state = simulateQuadcopter(state, uApply, dt, dyn);
        state = enforceDynamicEnvelope(state, dyn);
    else
        state = denormalizePrediction(yPred, yOffset, yScale, p);
        state = enforceDynamicEnvelope(state, dyn);
    end

    yNorm = (state - yOffset) ./ yScale;
    uNormApplied = (uApply - uOffset) ./ uScale;
    uHist = [uHist(:, 2:end), uNormApplied];
    yHist = [yHist(:, 2:end), yNorm];

    ref = buildPhysicalReference(path, pathIdx, dt, p);
    log.time(k) = (k - 1) * dt;
    log.state(:, k) = state;
    log.ref(:, k) = ref;
    log.u(:, k) = uApply;
    log.success(k) = info.success;
    log.pred(:, k) = denormalizePrediction(yPred, yOffset, yScale, p);
    log.pathIdx(k) = pathIdx;
    log.plannerState(k) = plannedMission.stateHistory(pathIdx);
    log.altitudeState(k) = plannedMission.altHistory(pathIdx);
    log.obstacleDetected(k) = plannedMission.obstacleDetected(pathIdx);
    log.plannerYaw(k) = plannedMission.yawHistory(pathIdx);
    log.err(k) = norm(state(1:3) - ref(1:3));

    if mod(k, 30) == 0 || k == 1
        fprintf('Time: %.1f s | Ref %d/%d | Pos: [%.2f %.2f %.2f] | Planner: [%.2f %.2f %.2f] | FSM %d/%d | QP: %d\n', ...
            log.time(k), pathIdx, size(path, 1), ...
            state(1), state(2), state(3), ref(1), ref(2), ref(3), ...
            log.plannerState(k), log.altitudeState(k), info.success);
    end

    if pathIdx >= size(path, 1) && norm(state(1:3) - scenario.B(:)) < 0.45
        log = trimLog(log, k);
        break;
    end
end

result = buildResult(log, plannedMission, scenario, meta, model);
validateResult(result);

fig2d = plotLabyrinth2D(result);
fig3d = plotLabyrinth3D(result);
figState = plotDynamicStates(result);

png2d = fullfile(resultsDir, 'LabyrinthDynamicDeePC_2d.png');
png3d = fullfile(resultsDir, 'LabyrinthDynamicDeePC_3d.png');
pngState = fullfile(resultsDir, 'LabyrinthDynamicDeePC_states.png');
matFile = fullfile(resultsDir, 'LabyrinthDynamicDeePC_result.mat');

saveFigure(fig2d, png2d);
saveFigure(fig3d, png3d);
saveFigure(figState, pngState);
save(matFile, 'result');

fprintf('\n====================================================\n');
fprintf('  Dynamic DeePC Labyrinth Test Complete\n');
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
fprintf('  Planner detect:      %d\n', result.metrics.usedPlannerDetection);
fprintf('  Planner orient:      %d\n', result.metrics.usedPlannerOrient);
fprintf('  Planner translate:   %d\n', result.metrics.usedPlannerTranslate);
fprintf('  Ceiling respected:   %d\n', result.metrics.ceilingRespected);
fprintf('  Collision-free:      %d\n', result.metrics.collisionFree);
fprintf('  Saved 2D:            %s\n', png2d);
fprintf('  Saved 3D:            %s\n', png3d);
fprintf('  Saved states:        %s\n', pngState);
fprintf('  Saved MAT:           %s\n', matFile);

%% Simulation helpers
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
    log.plannerYaw = zeros(1, n);
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

function plannerParams = makeLabyrinthPlannerParams(s, dyn)
    plannerParams = struct();
    plannerParams.Ts = dyn.dt;
    plannerParams.drone_radius = s.droneRadius;
    plannerParams.max_yaw_rate = pi;
    plannerParams.fov_mode = 'corridor';
    plannerParams.fov_angle = deg2rad(105);
    plannerParams.detection_range = 0.95;
    plannerParams.safe_distance = 0.42;
    plannerParams.corridor_width = 0.22;
    plannerParams.corridor_length = 0.80;
    plannerParams.enable_altitude = true;
    plannerParams.altitude_step = 0.45;
    plannerParams.min_altitude = 0.52;
    plannerParams.max_altitude = s.zCeiling - 0.03;
    plannerParams.height_limit_tolerance = 0.01;
    plannerParams.align_tolerance = deg2rad(8);
    plannerParams.goal_tolerance = 0.30;
    plannerParams.step_multiplier = 1.15;
    plannerParams.setpoint_gain = 0.35;
    plannerParams.min_obstacle_step_dist = 0.34;
    plannerParams.min_maneuver_step = 0.30;
    plannerParams.max_translate_step = 0.11;
    plannerParams.max_escape_step = 0.12;
    plannerParams.stuck_threshold = 30;
    plannerParams.stuck_escape_dist = 0.42;
    plannerParams.orient_timeout = 20;
end

function planned = planLabyrinthMissionReference(s, plannerParams, obstacles)
    mission = s.missionSetpoints;
    pos = s.A(:);
    yaw = pi / 2;
    maxSpeed = 0.75;
    maxStepsPerGoal = 500;

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
            [setpoint, yawCmd, planner, info] = ...
                pathPlannerStep(planner, pos, yaw, obstacles);
            yaw = wrapToPiLocal(yawCmd);

            err = setpoint - pos;
            step = min(norm(err), maxSpeed * plannerParams.Ts);
            if norm(err) > 1e-9
                pos = pos + (step / norm(err)) * err;
            end
            pos(3) = max(plannerParams.min_altitude, ...
                min(plannerParams.max_altitude, pos(3)));

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

    if norm(path(end, :) - s.B) > 1e-9
        path(end + 1, :) = s.B; %#ok<AGROW>
        stateHistory(end + 1) = stateHistory(end); %#ok<AGROW>
        altHistory(end + 1) = altHistory(end); %#ok<AGROW>
        obstacleDetected(end + 1) = false; %#ok<AGROW>
        yawHistory(end + 1) = yawHistory(end); %#ok<AGROW>
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
        yaw = pi / 2;
    else
        yaw = atan2(d(2), d(1));
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
        ref = buildPhysicalReference(path, idxJ, dt, p);
        rMat(:, j) = (ref - yOffset) ./ yScale;
    end
    r = rMat(:);
end

function ref = buildPhysicalReference(path, idx, dt, p)
    posRef = path(idx, :)';
    if idx >= size(path, 1)
        velRef = zeros(3, 1);
    else
        velRef = (path(idx + 1, :) - path(idx, :))' / max(dt, 1e-9);
    end
    n = norm(velRef);
    if n > 0.90
        velRef = velRef * (0.90 / n);
    end
    ref = zeros(p, 1);
    ref(1:3) = posRef;
    ref(4:6) = velRef;
    ref(9) = pi / 2;
end

function yPredPhys = denormalizePrediction(yPred, yOffset, yScale, p)
    if isempty(yPred)
        yPredPhys = zeros(p, 1);
    else
        yPredPhys = yPred(1:p) .* yScale + yOffset;
    end
end

function state = enforceDynamicEnvelope(state, dyn)
    state(1:12) = max(dyn.stateMin(:), min(dyn.stateMax(:), state(1:12)));
    state(9) = wrapToPiLocal(state(9));
    for i = 1:3
        if state(i) <= dyn.stateMin(i) + 1e-12 && state(i + 3) < 0
            state(i + 3) = 0;
        elseif state(i) >= dyn.stateMax(i) - 1e-12 && state(i + 3) > 0
            state(i + 3) = 0;
        end
    end
end

function result = buildResult(log, plannedMission, scenario, meta, model)
    result = struct();
    result.scenario = scenario;
    result.mission = scenario.missionSetpoints;
    result.plannedReference = plannedMission.path;
    result.planner = plannedMission;
    result.path = log.ref(1:3, :)';
    result.log = log;
    result.meta = meta;
    result.plantMode = 'behavioral';
    result.modelDims = model.dims;
    result.metrics = computeMetrics(log, scenario, plannedMission);
end

function metrics = computeMetrics(log, s, plannedMission)
    traj = log.state(1:3, :)';
    err = sqrt(sum((log.state(1:3, :) - log.ref(1:3, :)).^2, 1));

    metrics = struct();
    metrics.qpSuccessRate = mean(log.success);
    metrics.positionRmse = sqrt(mean(err.^2));
    metrics.finalError = norm(log.state(1:3, end) - s.B(:));
    metrics.maxError = max(err);
    metrics.minClearance = minObstacleClearance(traj, s);
    metrics.minWallClearance = labyrinthMinWallClearance(traj, s);
    metrics.hoverReached = any(traj(:, 3) >= 0.95 * s.zHover);
    metrics.usedRightBypass = any(traj(:, 1) > 0.24 & traj(:, 1) < 1.12 & ...
        traj(:, 2) > 2.0 & traj(:, 2) < 4.0);
    metrics.usedAltitudeUp = max(traj(:, 3)) > s.zHover + 0.28;
    metrics.usedAltitudeDown = min(traj(traj(:, 2) > 1.0, 3)) < s.zHover - 0.22;
    metrics.usedLeftBypass = any(traj(:, 1) < 5.86 & traj(:, 1) > 4.85 & ...
        traj(:, 2) > 2.2 & traj(:, 2) < 4.1);
    metrics.usedPlannerOrient = any(plannedMission.stateHistory == 3);
    metrics.usedPlannerTranslate = any(plannedMission.stateHistory == 4);
    metrics.usedPlannerDetection = any(plannedMission.obstacleDetected);
    metrics.usedPlannerAltitudeUp = any(plannedMission.altHistory == 1);
    metrics.usedPlannerAltitudeDown = any(plannedMission.altHistory == 2);
    metrics.ceilingRespected = all(traj(:, 3) < s.zCeiling);
    metrics.floorRespected = all(traj(:, 3) >= -1e-9);
    metrics.collisionFree = metrics.minClearance > -0.03;
    metrics.wallCollisionFree = metrics.minWallClearance > 0.03;
end

function validateResult(result)
    m = result.metrics;
    assert(m.qpSuccessRate > 0.95, 'LabyrinthDynamicDeePC:QpFailures', ...
        'The dynamic DeePC QP did not solve reliably enough.');
    assert(m.finalError < 0.65, 'LabyrinthDynamicDeePC:GoalMissed', ...
        'The dynamic DeePC trajectory did not reach the final labyrinth point.');
    assert(m.positionRmse < 0.75, 'LabyrinthDynamicDeePC:TrackingError', ...
        'The dynamic DeePC tracking error is too high for this test.');
    assert(m.hoverReached, 'LabyrinthDynamicDeePC:NoHover', ...
        'The quadrotor did not take off and reach hover.');
    assert(m.usedRightBypass && m.usedLeftBypass, 'LabyrinthDynamicDeePC:NoLateralAvoidance', ...
        'The lateral avoidance sections were not exercised.');
    assert(m.usedAltitudeUp && m.usedAltitudeDown, 'LabyrinthDynamicDeePC:NoAltitudeAvoidance', ...
        'The altitude avoidance sections were not exercised.');
    assert(m.usedPlannerOrient && m.usedPlannerTranslate && m.usedPlannerDetection, ...
        'LabyrinthDynamicDeePC:PlannerNotUsed', ...
        'The path-planner state machine did not actively generate avoidance maneuvers.');
    assert(m.ceilingRespected && m.floorRespected, 'LabyrinthDynamicDeePC:AltitudeLimit', ...
        'The dynamic DeePC path violated the altitude envelope.');
    assert(m.collisionFree, 'LabyrinthDynamicDeePC:Collision', ...
        'The dynamic DeePC path collided with an obstacle.');
    if ~m.wallCollisionFree
        [~, sampleIdx, wallIdx] = labyrinthMinWallClearance( ...
            result.log.state(1:3, :)', result.scenario);
        p = result.log.state(1:3, sampleIdx);
        fprintf('  Wall collision diagnostic: sample=%d wall=%d pos=[%.3f %.3f %.3f], clearance=%.3f\n', ...
            sampleIdx, wallIdx, p(1), p(2), p(3), m.minWallClearance);
    end
    assert(m.wallCollisionFree, 'LabyrinthDynamicDeePC:WallCollision', ...
        'The dynamic DeePC path collided with a labyrinth wall.');
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
    fig = figure('Name', 'Dynamic DeePC Labyrinth 2D', ...
        'Position', [80 80 980 760], 'Color', [0.06 0.06 0.06]);
    hold on; grid on; axis equal;
    styleDarkAxes(gca);
    drawCorridorWalls2D(s);
    drawObstacles2D(s);
    plot(result.path(:, 1), result.path(:, 2), 'r--', ...
        'LineWidth', 1.5, 'DisplayName', 'Reference path');
    plot(result.log.state(1, :), result.log.state(2, :), 'b-', ...
        'LineWidth', 2.0, 'DisplayName', 'Dynamic DeePC path');
    plot(s.setpoints(:, 1), s.setpoints(:, 2), 'rx', ...
        'MarkerSize', 8, 'LineWidth', 1.6, 'DisplayName', 'Setpoints');
    plot(s.A(1), s.A(2), 'go', 'MarkerFaceColor', 'g', ...
        'MarkerSize', 8, 'DisplayName', 'A start');
    plot(s.B(1), s.B(2), 'ko', 'MarkerFaceColor', [0.1 0.1 0.1], ...
        'MarkerSize', 8, 'DisplayName', 'B exit');
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Dynamic DeePC: physical thrust/torque tracking in labyrinth');
    xlim(s.axis2D(1:2));
    ylim(s.axis2D(3:4));
    legend('Location', 'eastoutside');
end

function fig = plotLabyrinth3D(result)
    s = result.scenario;
    fig = figure('Name', 'Dynamic DeePC Labyrinth 3D', ...
        'Position', [110 90 1050 760], 'Color', [0.06 0.06 0.06]);
    hold on; grid on; axis vis3d;
    styleDarkAxes(gca);
    drawCorridorWalls3D(s);
    drawObstacles3D(s);
    drawCeiling(s);
    plot3(result.path(:, 1), result.path(:, 2), result.path(:, 3), ...
        'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference path');
    plot3(result.log.state(1, :), result.log.state(2, :), result.log.state(3, :), ...
        'b-', 'LineWidth', 2.0, 'DisplayName', 'Dynamic DeePC path');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Dynamic DeePC labyrinth: 12-state quadrotor model');
    xlim(s.axis3D(1:2));
    ylim(s.axis3D(3:4));
    zlim(s.axis3D(5:6));
    view(42, 28);
    daspect([1 1 0.6]);
    legend('Location', 'eastoutside');
end

function fig = plotDynamicStates(result)
    t = result.log.time;
    fig = figure('Name', 'Dynamic DeePC States', ...
        'Position', [120 70 1120 860], 'Color', 'w');
    subplot(4, 1, 1);
    plot(t, result.log.ref(1:3, :)', '--', 'LineWidth', 1.2); hold on;
    plot(t, result.log.state(1:3, :)', '-', 'LineWidth', 1.4);
    grid on; ylabel('Position (m)');
    legend('x ref', 'y ref', 'z ref', 'x', 'y', 'z', 'Location', 'eastoutside');
    title('Dynamic DeePC state tracking');

    subplot(4, 1, 2);
    plot(t, rad2deg(result.log.state(7:9, :))', 'LineWidth', 1.2);
    grid on; ylabel('Euler (deg)');
    legend('roll', 'pitch', 'yaw', 'Location', 'eastoutside');

    subplot(4, 1, 3);
    stairs(t, result.log.plannerState, 'LineWidth', 1.3); hold on;
    stairs(t, result.log.altitudeState + 8, 'LineWidth', 1.3);
    grid on; ylabel('Planner FSM');
    ylim([-0.5, 12]);
    yticks([1 2 3 4 5 6 7 8 9 10 11]);
    yticklabels({'ALIGN','GOAL','ORIENT','TRANS','ESC','AT','COL','ALT0','UP','DOWN','HLIM'});
    legend('lateral', 'altitude + 8', 'Location', 'eastoutside');

    subplot(4, 1, 4);
    plot(t, result.log.u', 'LineWidth', 1.2);
    grid on; xlabel('Time (s)'); ylabel('Input');
    legend('T', 'tau_x', 'tau_y', 'tau_z', 'Location', 'eastoutside');
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

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
