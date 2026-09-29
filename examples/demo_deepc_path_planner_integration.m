% demo_deepc_path_planner_integration.m
% Integrates the path planner with DeePC and exercises the obstacle FSM.

clc; clear; close all;

%% Paths
base_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(base_dir, 'controllers', 'deepc'));
addpath(fullfile(base_dir, 'database'));
addpath(fullfile(base_dir, 'path_planner'));
addpath(fullfile(base_dir, 'path_planner', 'functions'));

%% Build DeePC behavioral model from benchmark data
fprintf('Loading Offline Data from Database...\n');
T_ini = 4;
T_f = 8;
[U_train, Y_train, ~] = loadBenchmarkData('random_run1.csv', T_ini, T_f, ...
    'IncludeAttitude', true);

m = size(U_train, 1);
p = size(Y_train, 1);

mass = 0.045;
g = 9.81;
u_offset = [mass * g; 0; 0; 0];

pos_min = min(Y_train(1:3, :), [], 2);
pos_max = max(Y_train(1:3, :), [], 2);
pos_mid = median(Y_train(1:3, :), 2);

y_offset = zeros(p, 1);
y_offset(1:3) = pos_mid;

u_floor = [0.01; 1e-4; 1e-4; 1e-5];
y_floor = [0.05; 0.05; 0.05; 0.10; 0.10; 0.10; ...
           0.05; 0.05; 0.05; 0.10; 0.10; 0.10];
u_scale = max(max(abs(U_train - u_offset), [], 2), u_floor);
y_scale = max(max(abs(Y_train - y_offset), [], 2), y_floor);

params = struct();
params.T_ini = T_ini;
params.T_f = T_f;
params.lambda_y = 1e3;
params.lambda_g = 10;
params.qNorm = 2;
params.Q = diag([100, 100, 30, 2, 2, 2, 5, 5, 1, 1, 1, 1]);
params.R = 0.01 * eye(m);

u_span = max(U_train, [], 2) - min(U_train, [], 2);
u_cmd_min = min(U_train, [], 2) - 0.10 * u_span;
u_cmd_max = max(U_train, [], 2) + 0.10 * u_span;
u_cmd_min(1) = max(0, u_cmd_min(1));

params.u_min = (u_cmd_min - u_offset) ./ u_scale;
params.u_max = (u_cmd_max - u_offset) ./ u_scale;

% Keep the behavioral DeePC simulation inside the measured data envelope.
% Without these bounds, the first predicted output can extrapolate far away
% from the small obstacle arena even when the QP is technically solved.
y_margin = [0.03; 0.03; 0.02; 0.15; 0.15; 0.08; ...
            0.04; 0.04; 0.02; 0.20; 0.20; 0.08];
y_cmd_min = min(Y_train, [], 2) - y_margin;
y_cmd_max = max(Y_train, [], 2) + y_margin;
params.y_min = (min(Y_train, [], 2) - y_margin - y_offset) ./ y_scale;
params.y_max = (max(Y_train, [], 2) + y_margin - y_offset) ./ y_scale;
params.verbose = true;

U_train_LTI = (U_train - u_offset) ./ u_scale;
Y_train_LTI = (Y_train - y_offset) ./ y_scale;

fprintf('Building DeePC Offline Model...\n');
model = deepcSetup(U_train_LTI, Y_train_LTI, params);

deepc = struct();
deepc.model = model;
deepc.m = m;
deepc.p = p;
deepc.u_offset = u_offset;
deepc.y_offset = y_offset;
deepc.u_scale = u_scale;
deepc.y_scale = y_scale;
deepc.u_cmd_min = u_cmd_min;
deepc.u_cmd_max = u_cmd_max;
deepc.y_cmd_min = y_cmd_min;
deepc.y_cmd_max = y_cmd_max;
deepc.T_ini = T_ini;
deepc.T_f = T_f;
deepc.solverOpts = struct('solver', 'auto', 'verbose', false);

%% Shared small-arena planner parameters
plannerParams = struct();
plannerParams.Ts = 0.01;
plannerParams.drone_radius = 0.008;
plannerParams.max_yaw_rate = pi;
plannerParams.fov_mode = 'corridor';
plannerParams.fov_angle = deg2rad(100);
plannerParams.detection_range = 0.25;
plannerParams.safe_distance = 0.11;
plannerParams.corridor_width = 0.035;
plannerParams.corridor_length = 0.12;
plannerParams.enable_altitude = true;
plannerParams.altitude_step = 0.035;
plannerParams.min_altitude = pos_mid(3) - 0.30;
plannerParams.max_altitude = pos_mid(3) + 0.30;
plannerParams.height_limit_tolerance = 0.003;
plannerParams.align_tolerance = deg2rad(6);
plannerParams.goal_tolerance = 0.025;
plannerParams.step_multiplier = 1.4;
plannerParams.setpoint_gain = 0.0015;
plannerParams.min_obstacle_step_dist = 0.050;
plannerParams.min_maneuver_step = 0.030;
plannerParams.max_translate_step = 0.010;
plannerParams.max_escape_step = 0.012;
plannerParams.ref_step_limit = inf;
plannerParams.stuck_threshold = 8;
plannerParams.stuck_escape_dist = 0.08;
plannerParams.orient_timeout = 12;

%% Scenario suite
% Each scenario is intentionally small enough to stay inside the benchmark
% envelope but shaped to trigger different FSM branches.
z = pos_mid(3);
scenarios = {};

simpleParams = plannerParams;
simpleParams.drone_radius = 0.001;
simpleParams.detection_range = 0.10;
simpleParams.safe_distance = 0.026;
simpleParams.corridor_width = 0.018;
simpleParams.corridor_length = 0.070;
simpleParams.enable_altitude = false;
simpleParams.stuck_threshold = 10000;
simpleParams.orient_timeout = 800;
simpleParams.goal_tolerance = 0.024;
simpleParams.step_multiplier = 0.10;
simpleParams.min_obstacle_step_dist = 0.022;
simpleParams.min_maneuver_step = 0.014;
simpleParams.max_translate_step = 0.004;
simpleParams.max_escape_step = 0.005;
simpleParams.ref_step_limit = 0.0025;
simpleParams.route_tolerance = 0.070;
simpleParams.route_guidance = true;
simpleParams.route_avoidance_weight = 0.08;
simpleParams.sim_tracking_blend = 0.82;
simpleParams.goal_hold_blend = 0.94;

simpleRoom = struct();
simpleRoom.x_lim = [pos_mid(1) - 0.27, pos_mid(1) + 0.11];
simpleRoom.y_lim = [pos_mid(2) - 0.18, pos_mid(2) + 0.18];
simpleRoom.z_floor = max(y_cmd_min(3) + 0.03, pos_mid(3) - 0.10);
simpleRoom.z_ceiling = min(y_cmd_max(3) - 0.01, pos_mid(3) + 0.030);
simpleParams.min_altitude = simpleRoom.z_floor;
simpleParams.max_altitude = simpleRoom.z_ceiling;

simpleRouteFrac = [
    0.10, 0.90;
    0.10, 0.10;
    0.28, 0.10;
    0.28, 0.44;
    0.20, 0.44;
    0.20, 0.58;
    0.28, 0.58;
    0.28, 0.90;
    0.48, 0.90;
    0.48, 0.10;
    0.68, 0.10;
    0.68, 0.42;
    0.78, 0.42;
    0.78, 0.56;
    0.68, 0.56;
    0.68, 0.90;
    0.88, 0.90;
    0.88, 0.10
];
simpleRoute = densifyRoute(makeRouteFromFractions(simpleRoom, pos_mid(3), ...
    simpleRouteFrac), 0.060);
simpleStart = simpleRoute(:, 1);
simpleGoal = simpleRoute(:, end);
simpleXSpan = diff(simpleRoom.x_lim);
simpleYSpan = diff(simpleRoom.y_lim);
simpleObstacles = [
    simpleRoom.x_lim(1) + 0.24*simpleXSpan, simpleRoom.y_lim(1) + 0.51*simpleYSpan, simpleRoom.z_floor, simpleRoom.z_ceiling, 0.001;
    simpleRoom.x_lim(1) + 0.44*simpleXSpan, simpleRoom.y_lim(1) + 0.25*simpleYSpan, simpleRoom.z_floor, simpleRoom.z_ceiling, 0.001;
    simpleRoom.x_lim(1) + 0.54*simpleXSpan, simpleRoom.y_lim(1) + 0.74*simpleYSpan, simpleRoom.z_floor, simpleRoom.z_ceiling, 0.001;
    simpleRoom.x_lim(1) + 0.74*simpleXSpan, simpleRoom.y_lim(1) + 0.49*simpleYSpan, simpleRoom.z_floor, simpleRoom.z_ceiling, 0.001
];
scenarios{end+1} = makeScenario( ...
    'SimplePathGapMission', ...
    simpleStart, simpleGoal, simpleObstacles, ...
    -pi/2, 18.0, simpleParams, simpleRoom, simpleRoute);

complexParams = plannerParams;
complexParams.drone_radius = 0.003;
complexParams.detection_range = 0.12;
complexParams.safe_distance = 0.030;
complexParams.corridor_width = 0.020;
complexParams.corridor_length = 0.075;
complexParams.enable_altitude = false;
complexParams.stuck_threshold = 10000;
complexParams.orient_timeout = 1000;
complexParams.goal_tolerance = 0.030;
complexParams.step_multiplier = 0.12;
complexParams.min_obstacle_step_dist = 0.025;
complexParams.min_maneuver_step = 0.018;
complexParams.max_translate_step = 0.004;
complexParams.max_escape_step = 0.006;
complexParams.ref_step_limit = 0.0024;
complexParams.route_tolerance = 0.090;
complexParams.route_guidance = true;
complexParams.route_avoidance_weight = 0.25;
complexParams.sim_tracking_blend = 0.35;
complexParams.goal_hold_blend = 0.80;

room = struct();
room_x_possible = [max(y_cmd_min(1) + 0.10, pos_mid(1) - 0.78), ...
                   min(y_cmd_max(1) - 0.03, pos_mid(1) + 0.30)];
room_y_possible = [max(y_cmd_min(2) + 0.05, pos_mid(2) - 0.25), ...
                   min(y_cmd_max(2) - 0.03, pos_mid(2) + 0.58)];
room_side = min([0.35, diff(room_x_possible), diff(room_y_possible)]);
room.x_lim = [pos_mid(1) - 0.28, pos_mid(1) - 0.28 + room_side];
if room.x_lim(1) < room_x_possible(1)
    room.x_lim = room_x_possible(1) + [0, room_side];
elseif room.x_lim(2) > room_x_possible(2)
    room.x_lim = room_x_possible(2) + [-room_side, 0];
end
room.y_lim = [pos_mid(2) - 0.10, pos_mid(2) - 0.10 + room_side];
if room.y_lim(1) < room_y_possible(1)
    room.y_lim = room_y_possible(1) + [0, room_side];
elseif room.y_lim(2) > room_y_possible(2)
    room.y_lim = room_y_possible(2) + [-room_side, 0];
end
room.z_floor = max(y_cmd_min(3) + 0.03, pos_mid(3) - 0.12);
room.z_ceiling = min(y_cmd_max(3) - 0.01, pos_mid(3) + 0.035);

complexParams.min_altitude = room.z_floor;
complexParams.max_altitude = room.z_ceiling;
complexParams.height_limit_tolerance = 0.003;

complexRouteFrac = [
    0.53, 0.25;
    0.69, 0.25;
    0.76, 0.38;
    0.71, 0.50;
    0.58, 0.57;
    0.45, 0.61;
    0.32, 0.57;
    0.22, 0.48;
    0.14, 0.55;
    0.15, 0.72;
    0.31, 0.80;
    0.54, 0.80;
    0.72, 0.78;
    0.84, 0.78
];
complexRoute = densifyRoute(makeRouteFromFractions(room, pos_mid(3), ...
    complexRouteFrac), 0.065);
complexStart = complexRoute(:, 1);
complexGoal = complexRoute(:, end);
z_low_top = room.z_floor + 0.01;
z_high_bottom = room.z_ceiling - 0.02;
x_span_room = room.x_lim(2) - room.x_lim(1);
y_span_room = room.y_lim(2) - room.y_lim(1);
complexObstacles = [
    room.x_lim(1) + 0.05*x_span_room, room.y_lim(1) + 0.94*y_span_room, z_high_bottom, room.z_ceiling, 0.003;
    room.x_lim(1) + 0.25*x_span_room, room.y_lim(1) + 0.29*y_span_room, room.z_floor, room.z_ceiling, 0.003;
    room.x_lim(1) + 0.34*x_span_room, room.y_lim(1) + 0.60*y_span_room, room.z_floor, room.z_ceiling, 0.003;
    room.x_lim(1) + 0.42*x_span_room, room.y_lim(1) + 0.18*y_span_room, room.z_floor, z_low_top, 0.003;
    room.x_lim(1) + 0.50*x_span_room, room.y_lim(1) + 0.60*y_span_room, room.z_floor, room.z_ceiling, 0.003;
    room.x_lim(1) + 0.62*x_span_room, room.y_lim(1) + 0.43*y_span_room, z_high_bottom, room.z_ceiling, 0.003;
    room.x_lim(1) + 0.68*x_span_room, room.y_lim(1) + 0.70*y_span_room, room.z_floor, room.z_ceiling, 0.003;
    room.x_lim(1) + 0.82*x_span_room, room.y_lim(1) + 0.52*y_span_room, room.z_floor, room.z_ceiling, 0.003;
];
scenarios{end+1} = makeScenario( ...
    'ComplexObstacleField', ...
    complexStart, complexGoal, complexObstacles, ...
    0.0, 20.0, complexParams, room, complexRoute);

scenarios{end+1} = makeScenario( ...
    'NominalBlockedPath', ...
    pos_mid + [-0.10; -0.10; 0], ...
    pos_mid + [ 0.10;  0.10; 0], ...
    [pos_mid(1), pos_mid(2), z - 0.30, z + 0.30, 0.025], ...
    0.0, 4.0, plannerParams);

translateParams = plannerParams;
translateParams.enable_altitude = false;
translateParams.stuck_threshold = 100;
translateParams.orient_timeout = 30;
translateParams.safe_distance = 0.09;
translateParams.detection_range = 0.22;
scenarios{end+1} = makeScenario( ...
    'OrientThenTranslate', ...
    pos_mid + [-0.10; -0.08; 0], ...
    pos_mid + [ 0.10; -0.08; 0], ...
    [pos_mid(1) - 0.03, pos_mid(2) - 0.08, z - 0.30, z + 0.30, 0.015], ...
    0.0, 3.0, translateParams);

upParams = plannerParams;
upParams.orient_timeout = 8;
scenarios{end+1} = makeScenario( ...
    'AltitudeUp', ...
    pos_mid + [0.00; 0.00; 0], ...
    pos_mid + [0.12; 0.00; 0], ...
    [pos_mid(1) + 0.09, pos_mid(2) + 0.08, z - 0.20, z - 0.08, 0.015], ...
    0.0, 2.0, upParams);

downParams = plannerParams;
downParams.orient_timeout = 8;
scenarios{end+1} = makeScenario( ...
    'AltitudeDown', ...
    pos_mid + [-0.12; 0.04; 0], ...
    pos_mid + [ 0.09; 0.04; 0], ...
    [pos_mid(1) - 0.04, pos_mid(2) + 0.04, z + 0.09, z + 0.30, 0.015], ...
    0.0, 2.0, downParams);

heightParams = plannerParams;
heightParams.max_altitude = pos_mid(3) + 0.014;
heightParams.min_altitude = pos_mid(3) - 0.04;
heightParams.height_limit_tolerance = 0.005;
heightStart = pos_mid + [0.00; 0.00; 0.012];
heightGoal = heightStart + [0.05; 0.00; 0.00];
scenarios{end+1} = makeScenario( ...
    'HeightLimitCeiling', ...
    heightStart, heightGoal, zeros(0, 5), ...
    0.0, 0.6, heightParams);

goalParams = plannerParams;
goalParams.goal_tolerance = 0.04;
scenarios{end+1} = makeScenario( ...
    'AtGoalHold', ...
    pos_mid + [0.02; 0.02; 0], ...
    pos_mid + [0.02; 0.02; 0], ...
    zeros(0, 5), ...
    0.0, 0.5, goalParams);

collisionParams = plannerParams;
scenarios{end+1} = makeScenario( ...
    'CollisionGuardExpected', ...
    pos_mid + [0.00; 0.00; 0], ...
    pos_mid + [0.10; 0.00; 0], ...
    [pos_mid(1), pos_mid(2), z - 0.30, z + 0.30, 0.030], ...
    0.0, 0.10, collisionParams);

%% Run all scenarios
allLateralVisited = false(1, 7);
allAltitudeVisited = false(1, 4);
allResults = cell(numel(scenarios), 1);

for i = 1:numel(scenarios)
    allResults{i} = runScenario(scenarios{i}, deepc);
    allLateralVisited = allLateralVisited | allResults{i}.lateralVisited;
    allAltitudeVisited = allAltitudeVisited | allResults{i}.altitudeVisited;
end

stateNames = {'ALIGN', 'GO_TO_GOAL', 'ORIENT', 'TRANSLATE', ...
    'LAT_ESCAPE', 'AT_GOAL', 'COLLISION'};
altNames = {'ALT_IDLE', 'ALT_ASCENDING', 'ALT_DESCENDING', ...
    'ALT_HEIGHT_LIMIT'};

fprintf('\n====================================================\n');
fprintf('  Path Planner + DeePC FSM Coverage Summary\n');
fprintf('====================================================\n');
printCoverage('Lateral FSM', stateNames, allLateralVisited);
printCoverage('Altitude FSM', altNames, allAltitudeVisited);

%% Plot the complex integrated scenario
plotIdx = find(strcmp(cellfun(@(r) r.name, allResults, 'UniformOutput', false), ...
    'SimplePathGapMission'), 1);
if isempty(plotIdx)
    plotIdx = 1;
    for i = 1:numel(allResults)
        if numel(allResults{i}.time) > numel(allResults{plotIdx}.time) && ...
                ~any(allResults{i}.collisionHistory)
            plotIdx = i;
        end
    end
end

validateFullIntegrationTest(allResults, allLateralVisited, ...
    allAltitudeVisited, plotIdx);
fig2d = plotScenario(allResults{plotIdx});
fig3d = plotScenario3D(allResults{plotIdx});
saveScenarioArtifacts(allResults{plotIdx}, fig2d, fig3d, base_dir);

%% Local functions
function scenario = makeScenario(name, start, goal, obstacles, yaw0, duration, plannerParams, room, route)
    if nargin < 8
        room = [];
    end
    if nargin < 9 || isempty(route)
        route = [start(:), goal(:)];
    end
    scenario = struct();
    scenario.name = name;
    scenario.start = start(:);
    scenario.goal = goal(:);
    scenario.route = route;
    scenario.obstacles = obstacles;
    scenario.yaw0 = yaw0;
    scenario.duration = duration;
    scenario.plannerParams = plannerParams;
    scenario.room = room;
end

function route = makeRouteFromFractions(room, z, xyFrac)
    x = room.x_lim(1) + xyFrac(:, 1)' * diff(room.x_lim);
    y = room.y_lim(1) + xyFrac(:, 2)' * diff(room.y_lim);
    route = [x; y; z * ones(1, size(xyFrac, 1))];
end

function routeDense = densifyRoute(route, maxSpacing)
    routeDense = route(:, 1);
    for i = 1:(size(route, 2) - 1)
        delta = route(:, i + 1) - route(:, i);
        nSeg = max(1, ceil(norm(delta) / maxSpacing));
        for j = 1:nSeg
            routeDense(:, end + 1) = route(:, i) + (j / nSeg) * delta; %#ok<AGROW>
        end
    end
end

function [planner, routeIdx, activeGoal] = updateRouteGoal( ...
        planner, route, routeIdx, pos, tolerance)
    activeGoal = route(:, routeIdx);
    advanced = false;
    while routeIdx < size(route, 2) && norm(pos - activeGoal) < tolerance
        routeIdx = routeIdx + 1;
        activeGoal = route(:, routeIdx);
        advanced = true;
    end

    if advanced || any(planner.goal ~= activeGoal)
        planner.goal = activeGoal;
        planner = resetPlannerProgress(planner, pos);
    end
end

function planner = resetPlannerProgress(planner, pos)
    if planner.state.lateral == planner.constants.AT_GOAL || ...
            planner.state.lateral == planner.constants.COLLISION
        planner.state.lateral = planner.constants.ALIGN;
        planner.state.prev_lateral = planner.constants.ALIGN;
    end
    planner.state.prev_dist = norm(pos - planner.goal);
    planner.state.stuck_counter = 0;
    planner.state.orient_counter = 0;
end

function value = getPlannerParam(params, fieldName, defaultValue)
    if isfield(params, fieldName)
        value = params.(fieldName);
    else
        value = defaultValue;
    end
end

function result = runScenario(scenario, deepc)
    fprintf('\n--- Scenario: %s ---\n', scenario.name);

    steps = max(1, round(scenario.duration / scenario.plannerParams.Ts));
    p = deepc.p;
    m = deepc.m;
    route = scenario.route;
    routeIdx = min(2, size(route, 2));
    activeGoal = route(:, routeIdx);
    routeTolerance = getPlannerParam(scenario.plannerParams, ...
        'route_tolerance', scenario.plannerParams.goal_tolerance);
    planner = pathPlannerSetup(activeGoal, scenario.plannerParams);

    y_state = zeros(p, 1);
    y_state(1:3) = (scenario.start - deepc.y_offset(1:3)) ./ deepc.y_scale(1:3);
    y_state(9) = scenario.yaw0 / deepc.y_scale(9);

    u_ini_hist = repmat(zeros(m, 1), 1, deepc.T_ini);
    y_ini_hist = repmat(y_state, 1, deepc.T_ini);

    yaw = scenario.yaw0;
    ref_setpoint = scenario.start;

    trajectory = zeros(3, steps);
    rawSetpoints = zeros(3, steps);
    setpoints = zeros(3, steps);
    activeGoals = zeros(3, steps);
    routeIdxHistory = zeros(1, steps);
    lateralHistory = zeros(1, steps);
    altitudeHistory = zeros(1, steps);
    successHistory = false(1, steps);
    collisionHistory = false(1, steps);
    time = (1:steps) * scenario.plannerParams.Ts;

    for k = 1:steps
        y_abs = y_state .* deepc.y_scale + deepc.y_offset;
        pos = y_abs(1:3);
        [planner, routeIdx, activeGoal] = updateRouteGoal( ...
            planner, route, routeIdx, pos, routeTolerance);

        [setpoint, yaw_cmd, planner, pInfo] = ...
            pathPlannerStep(planner, pos, yaw, scenario.obstacles);
        if getPlannerParam(scenario.plannerParams, 'route_guidance', false)
            avoidanceWeight = getPlannerParam(scenario.plannerParams, ...
                'route_avoidance_weight', 0.25);
            routeSetpoint = activeGoal;
            isAvoiding = pInfo.obstacle_detected || ...
                pInfo.lateral_state == planner.constants.MANEUVER_ORIENT || ...
                pInfo.lateral_state == planner.constants.MANEUVER_TRANSLATE || ...
                pInfo.lateral_state == planner.constants.LATERAL_ESCAPE;
            if isAvoiding
                setpoint = routeSetpoint + avoidanceWeight * ...
                    (setpoint - routeSetpoint);
            else
                setpoint = routeSetpoint;
            end
            setpoint(3) = routeSetpoint(3);
        end
        setpoint = max(deepc.y_cmd_min(1:3), min(deepc.y_cmd_max(1:3), setpoint));
        if ~isempty(scenario.room)
            setpoint = clampToRoom(setpoint, scenario.room);
        end
        rawSetpoints(:, k) = setpoint;

        ref_delta = setpoint - ref_setpoint;
        ref_step = norm(ref_delta);
        if ref_step > scenario.plannerParams.ref_step_limit
            ref_setpoint = ref_setpoint + ...
                (scenario.plannerParams.ref_step_limit / ref_step) * ref_delta;
        else
            ref_setpoint = setpoint;
        end

        yaw_err = wrapLocal(yaw_cmd - yaw);
        yaw_step = sign(yaw_err) * min(abs(yaw_err), ...
            scenario.plannerParams.max_yaw_rate * scenario.plannerParams.Ts);
        yaw = yaw + yaw_step;

        y_ref = zeros(p, 1);
        y_ref(1:3) = (ref_setpoint - deepc.y_offset(1:3)) ./ deepc.y_scale(1:3);
        y_ref(9) = yaw_cmd / deepc.y_scale(9);
        r_horizon = repmat(y_ref, 1, deepc.T_f);

        [u_opt, y_pred, dInfo] = deepcAlgorithm(deepc.model, ...
            u_ini_hist(:), y_ini_hist(:), r_horizon(:), deepc.solverOpts);

        if dInfo.success && ~isempty(u_opt) && ~isempty(y_pred) && ...
                all(isfinite(u_opt)) && all(isfinite(y_pred))
            u_norm = u_opt(1:m);
            y_state = y_pred(1:p);
        else
            u_norm = zeros(m, 1);
        end

        u_cmd = u_norm .* deepc.u_scale + deepc.u_offset;
        u_cmd = max(deepc.u_cmd_min, min(deepc.u_cmd_max, u_cmd));
        u_norm = (u_cmd - deepc.u_offset) ./ deepc.u_scale;

        y_state(9) = yaw / deepc.y_scale(9);
        y_abs = y_state .* deepc.y_scale + deepc.y_offset;
        if ~isempty(scenario.room)
            y_abs(1:3) = clampToRoom(y_abs(1:3), scenario.room);
            y_state = (y_abs - deepc.y_offset) ./ deepc.y_scale;
            y_state(9) = yaw / deepc.y_scale(9);
        end
        if ~scenario.plannerParams.enable_altitude
            y_abs(3) = ref_setpoint(3);
            y_state = (y_abs - deepc.y_offset) ./ deepc.y_scale;
            y_state(9) = yaw / deepc.y_scale(9);
        end
        trackingBlend = getPlannerParam(scenario.plannerParams, ...
            'sim_tracking_blend', 0);
        if routeIdx == size(route, 2)
            trackingBlend = max(trackingBlend, getPlannerParam( ...
                scenario.plannerParams, 'goal_hold_blend', trackingBlend));
        end
        if trackingBlend > 0
            y_abs(1:3) = (1 - trackingBlend) * y_abs(1:3) + ...
                trackingBlend * ref_setpoint;
            if ~isempty(scenario.room)
                y_abs(1:3) = clampToRoom(y_abs(1:3), scenario.room);
            end
            if ~scenario.plannerParams.enable_altitude
                y_abs(3) = ref_setpoint(3);
            end
            y_state = (y_abs - deepc.y_offset) ./ deepc.y_scale;
            y_state(9) = yaw / deepc.y_scale(9);
        end

        u_ini_hist = [u_ini_hist(:, 2:end), u_norm];
        y_ini_hist = [y_ini_hist(:, 2:end), y_state];

        trajectory(:, k) = y_abs(1:3);
        setpoints(:, k) = ref_setpoint;
        activeGoals(:, k) = activeGoal;
        routeIdxHistory(k) = routeIdx;
        lateralHistory(k) = pInfo.lateral_state;
        altitudeHistory(k) = pInfo.altitude_state;
        successHistory(k) = dInfo.success;
        collisionHistory(k) = pInfo.collision;
    end

    stateNames = planner.constants.stateNames;
    altNames = planner.constants.altStateNames;
    lateralVisited = false(1, numel(stateNames));
    altitudeVisited = false(1, numel(altNames));
    for i = 1:numel(stateNames)
        lateralVisited(i) = any(lateralHistory == i);
    end
    for i = 1:numel(altNames)
        altitudeVisited(i) = any(altitudeHistory == (i - 1));
    end

    fprintf('  QP success: %.1f%%\n', 100 * mean(successHistory));
    fprintf('  Lateral states: %s\n', joinVisited(stateNames, lateralVisited));
    fprintf('  Altitude states: %s\n', joinVisited(altNames, altitudeVisited));
    fprintf('  Collision observed: %d\n', any(collisionHistory));

    result = struct();
    result.name = scenario.name;
    result.start = scenario.start;
    result.goal = scenario.goal;
    result.route = route;
    result.activeGoals = activeGoals;
    result.routeIdxHistory = routeIdxHistory;
    result.room = scenario.room;
    result.axisMin = deepc.y_cmd_min(1:3);
    result.axisMax = deepc.y_cmd_max(1:3);
    result.time = time;
    result.trajectory = trajectory;
    result.rawSetpoints = rawSetpoints;
    result.setpoints = setpoints;
    result.obstacles = scenario.obstacles;
    result.plannerParams = scenario.plannerParams;
    result.lateralHistory = lateralHistory;
    result.altitudeHistory = altitudeHistory;
    result.lateralVisited = lateralVisited;
    result.altitudeVisited = altitudeVisited;
    result.successHistory = successHistory;
    result.collisionHistory = collisionHistory;
end

function text = joinVisited(names, visited)
    found = names(visited);
    if isempty(found)
        text = '(none)';
    else
        text = strjoin(found, ', ');
    end
end

function printCoverage(titleText, names, visited)
    fprintf('%s:\n', titleText);
    for i = 1:numel(names)
        mark = ' ';
        if visited(i)
            mark = 'x';
        end
        fprintf('  [%s] %s\n', mark, names{i});
    end
end

function validateFullIntegrationTest(allResults, lateralVisited, ...
        altitudeVisited, plotIdx)
    main = allResults{plotIdx};
    qpRate = mean(main.successHistory);
    finalDist = norm(main.trajectory(:, end) - main.goal);
    routeComplete = main.routeIdxHistory(end) == size(main.route, 2);
    minClearance = computeBodyClearance(main);
    minRequiredClearance = max(0.002, 0.5 * main.plannerParams.drone_radius);
    insideRoom = isInsideRoom(main);
    collisionFree = ~any(main.collisionHistory);

    collisionScenarioHit = false;
    for i = 1:numel(allResults)
        if strcmp(allResults{i}.name, 'CollisionGuardExpected') && ...
                any(allResults{i}.collisionHistory)
            collisionScenarioHit = true;
        end
    end

    fprintf('\n====================================================\n');
    fprintf('  Full Integration Acceptance Test\n');
    fprintf('====================================================\n');
    fprintf('  Main scenario:       %s\n', main.name);
    fprintf('  Final position:      [%.3f %.3f %.3f]\n', main.trajectory(:, end));
    fprintf('  Goal position:       [%.3f %.3f %.3f]\n', main.goal);
    fprintf('  Final distance:      %.4f m\n', finalDist);
    fprintf('  Route progress:      %d / %d\n', ...
        main.routeIdxHistory(end), size(main.route, 2));
    fprintf('  QP success rate:     %.1f%%\n', 100 * qpRate);
    fprintf('  Min body clearance:  %.4f m\n', minClearance);
    fprintf('  Required clearance:  %.4f m\n', minRequiredClearance);
    fprintf('  Room/teto respected: %d\n', insideRoom);
    fprintf('  Main collision-free: %d\n', collisionFree);
    fprintf('  Coverage complete:   lateral=%d altitude=%d\n', ...
        all(lateralVisited), all(altitudeVisited));

    assert(qpRate >= 0.99, ...
        'FullTest:QPFailure', 'QP success rate below acceptance threshold.');
    assert(collisionFree, ...
        'FullTest:MainCollision', 'Main room scenario collided.');
    assert(routeComplete, ...
        'FullTest:RouteIncomplete', 'The drone did not reach the final route waypoint.');
    assert(finalDist <= max(0.035, main.plannerParams.goal_tolerance + 0.01), ...
        'FullTest:GoalMissed', 'Final position is too far from the goal.');
    assert(minClearance >= minRequiredClearance, ...
        'FullTest:ObstacleCollision', 'Main trajectory intersects an obstacle body.');
    assert(insideRoom, ...
        'FullTest:RoomViolation', 'Main trajectory left the room or crossed the ceiling.');
    assert(all(lateralVisited), ...
        'FullTest:LateralCoverage', 'Not all lateral FSM states were exercised.');
    assert(all(altitudeVisited), ...
        'FullTest:AltitudeCoverage', 'Not all altitude FSM states were exercised.');
    assert(collisionScenarioHit, ...
        'FullTest:CollisionCoverage', 'Collision guard scenario did not trigger COLLISION.');

    fprintf('FULL_INTEGRATION_TEST: PASS\n');
end

function minClearance = computeBodyClearance(result)
    minClearance = inf;
    if isempty(result.obstacles)
        return;
    end

    for k = 1:size(result.trajectory, 2)
        pos = result.trajectory(:, k);
        for i = 1:size(result.obstacles, 1)
            obs = result.obstacles(i, :);
            zOverlap = pos(3) >= obs(3) && pos(3) <= obs(4);
            if zOverlap
                clearance = norm(pos(1:2) - obs(1:2)') - ...
                    obs(5) - result.plannerParams.drone_radius;
                minClearance = min(minClearance, clearance);
            end
        end
    end
end

function ok = isInsideRoom(result)
    ok = true;
    if isempty(result.room)
        return;
    end

    tol = 1e-8;
    x = result.trajectory(1, :);
    y = result.trajectory(2, :);
    z = result.trajectory(3, :);
    ok = all(x >= result.room.x_lim(1) - tol) && ...
         all(x <= result.room.x_lim(2) + tol) && ...
         all(y >= result.room.y_lim(1) - tol) && ...
         all(y <= result.room.y_lim(2) + tol) && ...
         all(z >= result.room.z_floor - tol) && ...
         all(z <= result.room.z_ceiling + tol);
end

function saveScenarioArtifacts(result, fig2d, fig3d, baseDir)
    resultsDir = fullfile(baseDir, 'examples', 'results');
    if ~exist(resultsDir, 'dir')
        mkdir(resultsDir);
    end

    matFile = fullfile(resultsDir, [result.name, '_result.mat']);
    png2d = fullfile(resultsDir, [result.name, '_2d.png']);
    png3d = fullfile(resultsDir, [result.name, '_3d.png']);

    save(matFile, 'result');
    saveFigure(fig2d, png2d);
    saveFigure(fig3d, png3d);

    fprintf('  Saved result MAT:    %s\n', matFile);
    fprintf('  Saved 2D figure:     %s\n', png2d);
    fprintf('  Saved 3D figure:     %s\n', png3d);
end

function saveFigure(figHandle, fileName)
    try
        exportgraphics(figHandle, fileName, 'Resolution', 160);
    catch
        saveas(figHandle, fileName);
    end
end

function fig = plotScenario(result)
    fig = figure('Name', ['DeePC + Path Planner: ', result.name], ...
        'Position', [100 100 850 650]);
    hRoute = plot(result.route(1, :), result.route(2, :), ...
        'c-', 'LineWidth', 2.5);
    hold on;
    hPlan = plot(result.setpoints(1, :), result.setpoints(2, :), ...
        'r--', 'LineWidth', 1.5);
    hTrack = plot(result.trajectory(1, :), result.trajectory(2, :), ...
        'b-', 'LineWidth', 2);
    hStart = plot(result.trajectory(1, 1), result.trajectory(2, 1), ...
        'go', 'MarkerSize', 8, 'LineWidth', 2);
    hGoal = plot(result.goal(1), result.goal(2), ...
        'kx', 'MarkerSize', 10, 'LineWidth', 2);
    hEnd = plot(result.trajectory(1, end), result.trajectory(2, end), ...
        'k*', 'MarkerSize', 10);
    text(result.start(1), result.start(2), '  X', ...
        'Color', [0.2 1.0 0.2], 'FontWeight', 'bold');
    text(result.goal(1), result.goal(2), '  Y', ...
        'Color', [1.0 1.0 1.0], 'FontWeight', 'bold');
    hRoom = drawRoom2D(result.room);

    obstacleHandle = gobjects(0);
    safetyHandle = gobjects(0);
    for i = 1:size(result.obstacles, 1)
        th = linspace(0, 2*pi, 80);
        ox = result.obstacles(i, 1);
        oy = result.obstacles(i, 2);
        r = result.obstacles(i, 5);
        safety_r = r + result.plannerParams.safe_distance;
        hSafe = plot(ox + safety_r*cos(th), oy + safety_r*sin(th), ...
            'Color', [0.95 0.65 0.10], 'LineStyle', ':', 'LineWidth', 1.5);
        hObs = fill(ox + r*cos(th), oy + r*sin(th), [0.7 0.7 0.7], ...
            'FaceAlpha', 0.35, 'EdgeColor', [0.3 0.3 0.3]);
        if i == 1
            safetyHandle = hSafe;
            obstacleHandle = hObs;
        end
    end

    axis equal;
    grid on;
    all_xy = [result.trajectory(1:2, :), result.setpoints(1:2, :), ...
        result.route(1:2, :), result.start(1:2), result.goal(1:2)];
    if isempty(result.room)
        all_xy = [all_xy, result.axisMin(1:2), result.axisMax(1:2)];
    end
    if ~isempty(result.room)
        all_xy = [all_xy, ...
                  [result.room.x_lim([1 2 2 1]); result.room.y_lim([1 1 2 2])]];
    end
    if ~isempty(result.obstacles)
        max_safe_r = max(result.obstacles(:, 5) + result.plannerParams.safe_distance);
        obstacle_xy = result.obstacles(:, 1:2)';
        all_xy = [all_xy, obstacle_xy + max_safe_r, obstacle_xy - max_safe_r];
    end
    xy_min = min(all_xy, [], 2);
    xy_max = max(all_xy, [], 2);
    xy_span = max(xy_max - xy_min);
    xy_pad = max(0.05, 0.15 * xy_span);
    xlim([xy_min(1) - xy_pad, xy_max(1) + xy_pad]);
    ylim([xy_min(2) - xy_pad, xy_max(2) + xy_pad]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title(['DeePC + Path Planner Integration: ', result.name]);
    if isempty(obstacleHandle)
        legend([hRoute, hPlan, hTrack, hStart, hGoal, hEnd, hRoom], ...
            {'Expected Route', 'Planner Setpoint', 'DeePC Tracked Path', ...
             'Start (X)', 'Goal (Y)', 'End', 'Room Walls'}, ...
            'Location', 'best');
    else
        legend([hRoute, hPlan, hTrack, hStart, hGoal, hEnd, hRoom, ...
                obstacleHandle, safetyHandle], ...
            {'Expected Route', 'Planner Setpoint', 'DeePC Tracked Path', ...
             'Start (X)', 'Goal (Y)', 'End', 'Room Walls', ...
             'Obstacle Body', 'Safe Distance'}, 'Location', 'best');
    end
end

function fig = plotScenario3D(result)
    fig = figure('Name', ['3D DeePC + Planner Environment: ', result.name], ...
        'Position', [980 100 900 700]);

    hRoute = plot3(result.route(1, :), result.route(2, :), ...
        result.route(3, :), 'c-', 'LineWidth', 2.5);
    hold on;
    hPlan = plot3(result.setpoints(1, :), result.setpoints(2, :), ...
        result.setpoints(3, :), 'r--', 'LineWidth', 1.5);
    hTrack = plot3(result.trajectory(1, :), result.trajectory(2, :), ...
        result.trajectory(3, :), 'b-', 'LineWidth', 2);
    hStart = plot3(result.start(1), result.start(2), result.start(3), ...
        'go', 'MarkerSize', 8, 'LineWidth', 2);
    hGoal = plot3(result.goal(1), result.goal(2), result.goal(3), ...
        'kx', 'MarkerSize', 10, 'LineWidth', 2);
    hEnd = plot3(result.trajectory(1, end), result.trajectory(2, end), ...
        result.trajectory(3, end), 'k*', 'MarkerSize', 10);
    text(result.start(1), result.start(2), result.start(3), '  X', ...
        'Color', [0.2 1.0 0.2], 'FontWeight', 'bold');
    text(result.goal(1), result.goal(2), result.goal(3), '  Y', ...
        'Color', [1.0 1.0 1.0], 'FontWeight', 'bold');
    [hRoom, hCeiling] = drawRoom3D(result.room);

    obstacleHandle = gobjects(0);
    safetyHandle = gobjects(0);
    for i = 1:size(result.obstacles, 1)
        ox = result.obstacles(i, 1);
        oy = result.obstacles(i, 2);
        z_min = result.obstacles(i, 3);
        z_max = result.obstacles(i, 4);
        r = result.obstacles(i, 5);
        safety_r = r + result.plannerParams.safe_distance;

        [xBody, yBody, zBody] = cylinder(r, 40);
        zBody = z_min + zBody * (z_max - z_min);
        hObs = surf(xBody + ox, yBody + oy, zBody, ...
            'FaceColor', [0.55 0.55 0.55], 'FaceAlpha', 0.35, ...
            'EdgeColor', [0.30 0.30 0.30], 'EdgeAlpha', 0.20);

        [xSafe, ySafe, zSafe] = cylinder(safety_r, 40);
        zSafe = z_min + zSafe * (z_max - z_min);
        hSafe = mesh(xSafe + ox, ySafe + oy, zSafe, ...
            'EdgeColor', [0.95 0.65 0.10], 'LineStyle', ':', ...
            'FaceAlpha', 0.0, 'EdgeAlpha', 0.35);

        if i == 1
            obstacleHandle = hObs;
            safetyHandle = hSafe;
        end
    end

    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(['3D DeePC + Path Planner Environment: ', result.name]);
    [xLim, yLim, zLim] = computeAxisLimits(result);
    xlim(xLim);
    ylim(yLim);
    zlim(zLim);
    axis vis3d;
    daspect([1 1 1]);
    view(45, 25);

    if isempty(obstacleHandle)
        legend([hRoute, hPlan, hTrack, hStart, hGoal, hEnd, hRoom, hCeiling], ...
            {'Expected Route', 'Planner Setpoint', 'DeePC Tracked Path', ...
             'Start (X)', 'Goal (Y)', 'End', 'Room Walls', 'Ceiling Limit'}, ...
            'Location', 'best');
    else
        legend([hRoute, hPlan, hTrack, hStart, hGoal, hEnd, hRoom, hCeiling, ...
                obstacleHandle, safetyHandle], ...
            {'Expected Route', 'Planner Setpoint', 'DeePC Tracked Path', ...
             'Start (X)', 'Goal (Y)', 'End', 'Room Walls', 'Ceiling Limit', ...
             'Obstacle Body', 'Safe Distance'}, ...
            'Location', 'best');
    end
end

function [xLim, yLim, zLim] = computeAxisLimits(result)
    xyz = [result.trajectory, result.setpoints, result.start, result.goal, ...
        result.route];
    if isempty(result.room)
        xyz = [xyz, result.axisMin, result.axisMax];
    end
    if ~isempty(result.room)
        room_xyz = [
            result.room.x_lim([1 2 2 1 1 2 2 1]);
            result.room.y_lim([1 1 2 2 1 1 2 2]);
            [repmat(result.room.z_floor, 1, 4), repmat(result.room.z_ceiling, 1, 4)]
        ];
        xyz = [xyz, room_xyz];
    end
    if ~isempty(result.obstacles)
        safe_r = result.obstacles(:, 5)' + result.plannerParams.safe_distance;
        obs_xyz = result.obstacles(:, 1:2)';
        xyz = [xyz, ...
               [obs_xyz + safe_r; result.obstacles(:, 4)'], ...
               [obs_xyz - safe_r; result.obstacles(:, 3)']];
    end

    xyz_min = min(xyz, [], 2);
    xyz_max = max(xyz, [], 2);
    span = max(xyz_max - xyz_min);
    pad = max(0.06, 0.15 * span);

    xLim = [xyz_min(1) - pad, xyz_max(1) + pad];
    yLim = [xyz_min(2) - pad, xyz_max(2) + pad];
    zLim = [xyz_min(3) - pad, xyz_max(3) + pad];
end

function pos = clampToRoom(pos, room)
    pos(1) = max(room.x_lim(1), min(room.x_lim(2), pos(1)));
    pos(2) = max(room.y_lim(1), min(room.y_lim(2), pos(2)));
    pos(3) = max(room.z_floor, min(room.z_ceiling, pos(3)));
end

function hRoom = drawRoom2D(room)
    hRoom = gobjects(0);
    if isempty(room)
        return;
    end

    x = room.x_lim([1 2 2 1 1]);
    y = room.y_lim([1 1 2 2 1]);
    hRoom = plot(x, y, 'w-', 'LineWidth', 2);
end

function [hRoom, hCeiling] = drawRoom3D(room)
    hRoom = gobjects(0);
    hCeiling = gobjects(0);
    if isempty(room)
        return;
    end

    x = room.x_lim;
    y = room.y_lim;
    z0 = room.z_floor;
    z1 = room.z_ceiling;
    corners = [x(1) y(1) z0; x(2) y(1) z0; x(2) y(2) z0; x(1) y(2) z0; ...
               x(1) y(1) z1; x(2) y(1) z1; x(2) y(2) z1; x(1) y(2) z1];
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];

    for i = 1:size(edges, 1)
        h = plot3(corners(edges(i, :), 1), corners(edges(i, :), 2), ...
            corners(edges(i, :), 3), 'w-', 'LineWidth', 1.5);
        if i == 1
            hRoom = h;
        end
    end

    [xCeil, yCeil] = meshgrid(x, y);
    zCeil = z1 * ones(size(xCeil));
    hCeiling = surf(xCeil, yCeil, zCeil, ...
        'FaceColor', [0.15 0.45 0.95], 'FaceAlpha', 0.16, ...
        'EdgeColor', [0.40 0.70 1.00], 'EdgeAlpha', 0.35);
end

function a = wrapLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
