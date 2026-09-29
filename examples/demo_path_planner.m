%% demo_path_planner.m
%  Validates the path planner state machine across three distinct scenarios.
%
%  Each scenario uses a simple kinematic model (proportional chase) to
%  isolate the path planner's branching logic from any controller.
%  The goal is to verify that:
%    1. The lateral FSM transitions correctly
%    2. The parallel altitude FSM triggers when needed
%    3. Stuck detection and lateral escape work
%
%  Scenarios:
%    (A) Simple     — 3 obstacles, basic wall-follower cycle
%    (B) Mixed 3D   — low + floating obstacles, tests altitude changes
%    (C) Challenging — dense field, tests stuck detection + escape

clear; clc; close all;

fprintf('====================================================\n');
fprintf('  Path Planner Demo — 3 Scenarios\n');
fprintf('====================================================\n\n');

%% Add paths
plannerPath = fullfile(fileparts(mfilename('fullpath')), '..', 'path_planner');
addpath(plannerPath);
addpath(fullfile(plannerPath, 'functions'));

%% ========================================================================
%  Shared simulation engine
%  ========================================================================
%  A simple kinematic model that chases the setpoint.
%  NO controller (DeePC/MPC/etc.) is used — this is pure planner testing.

function results = runScenario(scenarioName, start, goal, obstacles, params, T_sim)
    fprintf('\n--- Scenario: %s ---\n', scenarioName);
    fprintf('  Start: [%.1f, %.1f, %.1f]\n', start);
    fprintf('  Goal:  [%.1f, %.1f, %.1f]\n', goal);
    fprintf('  Obstacles: %d\n', size(obstacles, 1));

    % Setup planner
    planner = pathPlannerSetup(goal, params);

    Ts = params.Ts;
    n_steps = round(T_sim / Ts);

    % State
    pos = start(:);
    yaw = atan2(goal(2) - start(2), goal(1) - start(1));
    vel = zeros(3, 1);

    % Storage
    trajectory    = zeros(3, n_steps);
    stateHistory  = zeros(1, n_steps);
    altHistory    = zeros(1, n_steps);
    yawHistory    = zeros(1, n_steps);
    distHistory   = zeros(1, n_steps);
    collisionPts  = [];

    % Kinematic chase gains
    Kp_pos = 3.0;   % Position proportional gain
    Kp_yaw = 2.0;   % Yaw proportional gain
    max_vel = 1.5;   % Maximum velocity [m/s]
    alpha   = 0.15;  % Velocity filter (smoothing)

    tic;
    for k = 1:n_steps
        %% Path planner step
        [setpoint, yaw_cmd, planner, info] = pathPlannerStep(planner, pos, yaw, obstacles);

        %% Simple kinematic model (chase the setpoint)
        pos_error = setpoint - pos;
        vel_cmd   = Kp_pos * pos_error;

        % Velocity saturation
        speed = norm(vel_cmd);
        if speed > max_vel
            vel_cmd = vel_cmd * (max_vel / speed);
        end

        % Low-pass filter on velocity
        vel = vel * (1 - alpha) + vel_cmd * alpha;

        % Update position
        pos = pos + vel * Ts;

        % Clamp altitude
        pos(3) = max(params.min_altitude, min(params.max_altitude, pos(3)));

        % Yaw tracking
        yaw_error = atan2(sin(yaw_cmd - yaw), cos(yaw_cmd - yaw));
        yaw = yaw + Kp_yaw * yaw_error * Ts;

        %% Store
        trajectory(:, k)  = pos;
        stateHistory(k)   = info.lateral_state;
        altHistory(k)     = info.altitude_state;
        yawHistory(k)     = yaw;
        distHistory(k)    = info.dist_to_goal;

        if info.collision
            collisionPts = [collisionPts, pos]; %#ok<AGROW>
        end

        %% Progress
        if mod(k, round(n_steps / 5)) == 0
            fprintf('  t=%5.1fs | %-12s | Alt: %-10s | Dist: %5.2fm\n', ...
                k * Ts, info.lateral_name, info.altitude_name, info.dist_to_goal);
        end

        %% Early termination
        if info.lateral_state == 6  % AT_GOAL
            trajectory    = trajectory(:, 1:k);
            stateHistory  = stateHistory(1:k);
            altHistory    = altHistory(1:k);
            yawHistory    = yawHistory(1:k);
            distHistory   = distHistory(1:k);
            fprintf('  ✓ GOAL REACHED at t=%.2fs\n', k * Ts);
            break;
        end

        if info.lateral_state == 7  % COLLISION
            trajectory    = trajectory(:, 1:k);
            stateHistory  = stateHistory(1:k);
            altHistory    = altHistory(1:k);
            yawHistory    = yawHistory(1:k);
            distHistory   = distHistory(1:k);
            fprintf('  ✗ COLLISION at t=%.2fs\n', k * Ts);
            break;
        end
    end
    simTime = toc;

    %% Metrics
    total_dist  = sum(vecnorm(diff(trajectory, 1, 2), 2, 1));
    direct_dist = norm(goal - start);
    efficiency  = direct_dist / max(total_dist, 1e-6) * 100;
    reached     = stateHistory(end) == 6;
    collided    = ~isempty(collisionPts);
    sim_seconds = size(trajectory, 2) * Ts;

    % State distribution
    stateNames = planner.constants.stateNames;
    nStates    = length(stateNames);
    statePct   = zeros(1, nStates);
    for s = 1:nStates
        statePct(s) = sum(stateHistory == s) / length(stateHistory) * 100;
    end

    %% Print metrics
    fprintf('\n  --- Results ---\n');
    fprintf('  Outcome:    %s\n', iff(reached, 'SUCCESS', iff(collided, 'COLLISION', 'TIMEOUT')));
    fprintf('  Sim time:   %.2fs (real: %.3fs)\n', sim_seconds, simTime);
    fprintf('  Path:       %.2fm (direct: %.2fm, efficiency: %.1f%%)\n', ...
        total_dist, direct_dist, efficiency);
    fprintf('  Collisions: %d\n', size(collisionPts, 2));
    fprintf('  State distribution:\n');
    for s = 1:nStates
        if statePct(s) > 0
            fprintf('    %-12s  %5.1f%%\n', stateNames{s}, statePct(s));
        end
    end

    % Check if altitude was used
    altUsed = any(altHistory ~= 0);
    fprintf('  Altitude FSM: %s\n', iff(altUsed, 'ACTIVE (3D avoidance used)', 'IDLE (2D only)'));

    %% Pack results
    results = struct();
    results.name         = scenarioName;
    results.trajectory   = trajectory;
    results.stateHistory = stateHistory;
    results.altHistory   = altHistory;
    results.yawHistory   = yawHistory;
    results.distHistory  = distHistory;
    results.collisionPts = collisionPts;
    results.obstacles    = obstacles;
    results.goal         = goal;
    results.start        = start;
    results.reached      = reached;
    results.collided     = collided;
    results.efficiency   = efficiency;
    results.totalDist    = total_dist;
    results.simTime      = sim_seconds;
    results.statePct     = statePct;
    results.altUsed      = altUsed;
    results.constants    = planner.constants;
    results.params       = params;
end

%% ========================================================================
%  SCENARIO A: Simple — Basic Wall-Follower Cycle
%  ========================================================================
%  Tests: ALIGN → GO_TO_GOAL → ORIENT → TRANSLATE → ALIGN → GO_TO_GOAL
%  Expected: Reaches goal with standard lateral maneuvers, no altitude needed

paramsA = struct();
paramsA.Ts = 0.01;
paramsA.fov_mode = 'corridor';
paramsA.enable_altitude = false;  % Force 2D to test lateral-only
paramsA.min_altitude = 0.3;
paramsA.max_altitude = 3.0;

startA = [0; 0; 1.0];
goalA  = [8; 8; 1.0];
obsA   = createObstacleEnvironment('simple');

resultsA = runScenario('A: Simple (2D lateral only)', startA, goalA, obsA, paramsA, 60);

%% ========================================================================
%  SCENARIO B: Mixed 3D — Altitude Control
%  ========================================================================
%  Tests: Parallel altitude FSM (ASCENDING / DESCENDING)
%  Expected: Drone flies over low obstacles & under floating obstacles

paramsB = struct();
paramsB.Ts = 0.01;
paramsB.fov_mode = 'corridor';
paramsB.enable_altitude = true;   % Enable 3D avoidance
paramsB.altitude_step = 0.5;
paramsB.min_altitude = 0.3;
paramsB.max_altitude = 2.5;

startB = [0; 0; 1.0];
goalB  = [8; 8; 1.0];
obsB   = createObstacleEnvironment('mixed_3d');

resultsB = runScenario('B: Mixed 3D (altitude enabled)', startB, goalB, obsB, paramsB, 80);

%% ========================================================================
%  SCENARIO C: Challenging — Stuck Detection + Escape
%  ========================================================================
%  Tests: Stuck counter, orient timeout, lateral escape, alternating direction
%  Expected: Drone gets stuck, triggers escape, eventually reaches goal

paramsC = struct();
paramsC.Ts = 0.01;
paramsC.fov_mode = 'dual';         % Test dual FOV mode
paramsC.enable_altitude = true;
paramsC.altitude_step = 0.5;
paramsC.stuck_threshold = 30;      % Trigger stuck faster for testing
paramsC.orient_timeout = 50;       % Lower timeout for testing
paramsC.min_altitude = 0.3;
paramsC.max_altitude = 2.5;

startC = [0; 0; 1.0];
goalC  = [8; 8; 1.0];
obsC   = createObstacleEnvironment('challenging');

resultsC = runScenario('C: Challenging (stuck + escape)', startC, goalC, obsC, paramsC, 120);

%% ========================================================================
%  CONSOLIDATED VISUALIZATION
%  ========================================================================
fprintf('\n====================================================\n');
fprintf('  Generating visualizations...\n');
fprintf('====================================================\n');

allResults = {resultsA, resultsB, resultsC};

for i = 1:3
    R = allResults{i};
    opts = struct();
    opts.Ts = R.params.Ts;
    opts.title = sprintf('Scenario %s', R.name);
    opts.collisionPts = R.collisionPts;
    opts.altHistory = R.altHistory;

    plotAvoidanceResults(R.trajectory, R.stateHistory, R.yawHistory, ...
        R.obstacles, R.goal, R.start, R.constants, opts);
end

%% ========================================================================
%  COMPARISON TABLE
%  ========================================================================
fprintf('\n====================================================\n');
fprintf('  COMPARISON TABLE\n');
fprintf('====================================================\n');
fprintf('  %-30s  %8s  %8s  %8s  %8s  %8s\n', ...
    'Scenario', 'Outcome', 'Time(s)', 'Path(m)', 'Eff(%)', 'AltUsed');
fprintf('  %s\n', repmat('-', 1, 82));

for i = 1:3
    R = allResults{i};
    fprintf('  %-30s  %8s  %8.2f  %8.2f  %7.1f%%  %8s\n', ...
        R.name, ...
        iff(R.reached, 'SUCCESS', iff(R.collided, 'COLLSN', 'TIMEOUT')), ...
        R.simTime, R.totalDist, R.efficiency, ...
        iff(R.altUsed, 'Yes', 'No'));
end

fprintf('\n  ✓ Demo complete.\n');

%% ========================================================================
%  HELPER
%  ========================================================================
function r = iff(cond, t, f)
    if cond, r = t; else, r = f; end
end
