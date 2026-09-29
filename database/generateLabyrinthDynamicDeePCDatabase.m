function dataFile = generateLabyrinthDynamicDeePCDatabase(varargin)
% GENERATELABYRINTHDYNAMICDEEPCDATABASE Create 6-DOF quadrotor data for DeePC.
%
% Inputs are physical [T; tau_x; tau_y; tau_z]. Outputs are the full
% 12-state quadrotor vector [x;y;z;vx;vy;vz;roll;pitch;yaw;wx;wy;wz].

    p = inputParser;
    addParameter(p, 'OutputDir', fullfile(fileparts(mfilename('fullpath')), 'labyrinth_dynamic'), @ischar);
    addParameter(p, 'NumSamples', 520, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Seed', 17, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'WriteCsv', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    if ~exist(opts.OutputDir, 'dir')
        mkdir(opts.OutputDir);
    end

    baseDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(baseDir, 'models'));

    scenario = labyrinthScenarioConfig();
    dyn = scenario.dynamic;
    rng(opts.Seed);

    N = round(opts.NumSamples);
    U = zeros(4, N);
    Y = zeros(12, N);
    X = zeros(12, N);
    clearance = zeros(1, N);
    wallClearance = zeros(1, N);
    safetyBuffer = 0.050;

    state = zeros(12, 1);
    state(1:3) = scenario.A(:);
    state(9) = pi / 2;

    route = buildDynamicExcitationRoute(scenario);
    routeIdx = 1;
    ditherPhase = 2 * pi * rand(4, 1);

    for k = 1:N
        routeIdx = advanceRouteIndex(route, routeIdx, state(1:3)', 0.34);
        ref = route(routeIdx, :)';
        yawRef = pi / 2;
        velRef = referenceVelocity(route, routeIdx, dyn.dt);

        uNom = dynamicReferenceController(state, ref, velRef, yawRef, dyn);
        uExc = excitationSignal(k, ditherPhase);
        u = clampInput(uNom + uExc, dyn);

        state = simulateQuadcopter(state, u, dyn.dt, dyn);
        state = enforceDynamicEnvelope(state, dyn);
        for guardIter = 1:2
            state = enforceObstacleClearance(state, scenario, safetyBuffer);
            state = enforceWallClearance(state, scenario, safetyBuffer);
        end
        clearance(k) = minObstacleClearance(state(1:3)', scenario);
        wallClearance(k) = labyrinthMinWallClearance(state(1:3)', scenario);

        noise = [
            0.0015 * randn(3, 1);
            0.0060 * randn(3, 1);
            0.0010 * randn(3, 1);
            0.0060 * randn(3, 1)
        ];

        U(:, k) = u;
        Y(:, k) = state + noise;
        X(:, k) = state;
    end

    t = (0:N - 1) * dyn.dt;

    data = struct();
    data.U = U;
    data.Y = Y;
    data.X = X;
    data.t = t;
    data.scenario = scenario;
    data.inputNames = scenario.dynamicInputNames;
    data.outputNames = scenario.dynamicOutputNames;
    data.meta = struct( ...
        'description', 'Labyrinth DeePC dynamic 6-DOF quadrotor database', ...
        'inputModel', '[T; tau_x; tau_y; tau_z]', ...
        'outputModel', '[x;y;z;vx;vy;vz;roll;pitch;yaw;wx;wy;wz]', ...
        'numSamples', N, ...
        'dt', dyn.dt, ...
        'seed', opts.Seed, ...
        'hoverThrust', dyn.hoverThrust, ...
        'obstacleSafetyBuffer', safetyBuffer, ...
        'minObstacleClearance', min(clearance), ...
        'minWallClearance', min(wallClearance));

    dataFile = fullfile(opts.OutputDir, 'labyrinth_dynamic_deepc_data.mat');
    save(dataFile, 'data');

    if opts.WriteCsv
        csvFile = fullfile(opts.OutputDir, 'labyrinth_dynamic_deepc_data.csv');
        writeDynamicCsv(csvFile, data);
    end

    fprintf('Generated dynamic labyrinth DeePC database:\n');
    fprintf('  MAT: %s\n', dataFile);
    if opts.WriteCsv
        fprintf('  CSV: %s\n', csvFile);
    end
    fprintf('  Samples: %d | dt: %.3f s | min clearance in excitation: %.3f m | min wall clearance: %.3f m\n', ...
        N, dyn.dt, data.meta.minObstacleClearance, data.meta.minWallClearance);
end

function route = buildDynamicExcitationRoute(s)
    if isfield(s, 'trainingSetpoints') && ~isempty(s.trainingSetpoints)
        points = s.trainingSetpoints;
    else
        points = s.setpoints;
    end
    base = resamplePolyline3D(points, 0.22);
    variants = zeros(0, 3);
    for i = 1:size(base, 1)
        variants(end + 1, :) = base(i, :); %#ok<AGROW>
        if i > 3 && i < size(base, 1) - 3
            p = base(i, :) + [0.10 * sin(0.37 * i), 0.10 * cos(0.29 * i), 0.08 * sin(0.19 * i)];
            p(1) = min(max(p(1), s.bounds.x(1) + 0.15), s.bounds.x(2) - 0.15);
            p(2) = min(max(p(2), s.bounds.y(1) + 0.15), s.bounds.y(2) - 0.15);
            p(3) = min(max(p(3), s.bounds.z(1) + 0.18), s.bounds.z(2) - 0.10);
            variants(end + 1, :) = p; %#ok<AGROW>
        end
    end
    route = variants;
end

function u = dynamicReferenceController(state, posRef, velRef, yawRef, dyn)
    pos = state(1:3);
    vel = state(4:6);
    eul = state(7:9);
    omega = state(10:12);

    kp = [2.60; 2.60; 3.35];
    kd = [2.85; 2.85; 2.45];
    acc = kp .* (posRef - pos) + kd .* (velRef - vel);
    acc(1:2) = limitVector(acc(1:2), 3.0);
    acc(3) = min(max(acc(3), -2.6), 3.0);

    yaw = eul(3);
    phiDes = (sin(yaw) * acc(1) - cos(yaw) * acc(2)) / dyn.g;
    thetaDes = (cos(yaw) * acc(1) + sin(yaw) * acc(2)) / dyn.g;
    phiDes = min(max(phiDes, -0.42), 0.42);
    thetaDes = min(max(thetaDes, -0.42), 0.42);

    thrust = dyn.mass * (dyn.g + acc(3)) / max(0.82, cos(phiDes) * cos(thetaDes));
    yawErr = wrapToPiLocal(yawRef - yaw);

    kpAng = [1.25e-4; 1.25e-4; 5.4e-5];
    kdAng = [2.4e-5; 2.4e-5; 1.4e-5];
    tau = kpAng .* ([phiDes; thetaDes; yawErr] - eul([1 2 3])) - kdAng .* omega;
    tau(3) = kpAng(3) * yawErr - kdAng(3) * omega(3);

    u = [thrust; tau];
    u = clampInput(u, dyn);
end

function uExc = excitationSignal(k, phase)
    uExc = [
        0.006 * sin(0.19 * k + phase(1)) + 0.003 * sin(0.047 * k);
        1.0e-6 * sin(0.31 * k + phase(2)) + 0.5e-6 * sin(0.071 * k);
        1.0e-6 * cos(0.27 * k + phase(3)) + 0.5e-6 * cos(0.063 * k);
        0.2e-6 * sin(0.23 * k + phase(4))
    ];
end

function u = clampInput(u, dyn)
    u = max(dyn.uMin(:), min(dyn.uMax(:), u(:)));
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

function state = enforceObstacleClearance(state, s, buffer)
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        if state(3) < obs(3) || state(3) > obs(4)
            continue;
        end

        delta = state(1:2) - obs(1:2)';
        dist = norm(delta);
        if dist < 1e-9
            delta = [1; 0];
            dist = 1;
        end

        radius = obs(5) + s.droneRadius;
        c = dist - radius;
        if c < buffer
            dir = delta / dist;
            state(1:2) = obs(1:2)' + (radius + buffer) * dir;

            radialVel = dot(state(4:5), dir);
            if radialVel < 0
                state(4:5) = state(4:5) - radialVel * dir;
            end
        end
    end
end

function state = enforceWallClearance(state, s, buffer)
    walls = labyrinthWallSegments(s);
    keepout = s.droneRadius + buffer;
    mazeCenter = [mean(s.axis2D(1:2)); mean(s.axis2D(3:4))];

    for i = 1:size(walls, 1)
        a = walls(i, 1:2)';
        b = walls(i, 3:4)';
        d = b - a;
        alpha = dot(state(1:2) - a, d) / max(dot(d, d), 1e-12);
        alpha = max(0, min(1, alpha));
        closest = a + alpha * d;

        delta = state(1:2) - closest;
        dist = norm(delta);
        if dist - s.droneRadius >= buffer
            continue;
        end

        if isExternalWall(walls(i, :), s) || dist < 1e-9
            dir = mazeCenter - closest;
        else
            dir = delta;
        end
        if norm(dir) < 1e-9
            dir = [-d(2); d(1)];
        end
        dir = dir / max(norm(dir), 1e-9);

        state(1:2) = closest + keepout * dir;
        wallVel = dot(state(4:5), dir);
        if wallVel < 0
            state(4:5) = state(4:5) - wallVel * dir;
        end
    end
end

function tf = isExternalWall(wall, s)
    xMin = min(min(s.wallSegments(:, [1 3])));
    xMax = max(max(s.wallSegments(:, [1 3])));
    yMin = min(min(s.wallSegments(:, [2 4])));
    yMax = max(max(s.wallSegments(:, [2 4])));

    isVertical = abs(wall(1) - wall(3)) < 1e-9;
    isHorizontal = abs(wall(2) - wall(4)) < 1e-9;
    tf = (isVertical && (abs(wall(1) - xMin) < 1e-9 || abs(wall(1) - xMax) < 1e-9)) || ...
         (isHorizontal && (abs(wall(2) - yMin) < 1e-9 || abs(wall(2) - yMax) < 1e-9));
end

function idx = advanceRouteIndex(route, idx, pos, radius)
    searchEnd = min(idx + 18, size(route, 1));
    window = route(idx:searchEnd, :);
    dist = sqrt(sum((window - pos).^2, 2));
    [~, localIdx] = min(dist);
    idx = idx + localIdx - 1;
    while idx < size(route, 1) && norm(route(idx, :) - pos) < radius
        idx = idx + 1;
    end
end

function vel = referenceVelocity(route, idx, dt)
    if idx >= size(route, 1)
        vel = zeros(3, 1);
        return;
    end
    vel = (route(min(idx + 1, size(route, 1)), :) - route(idx, :))' / dt;
    vel = limitVector(vel, 0.68);
end

function yaw = routeYaw(route, idx)
    idx2 = idx;
    while idx2 < size(route, 1) && norm(route(idx2, 1:2) - route(idx, 1:2)) < 1e-6
        idx2 = idx2 + 1;
    end
    d = route(idx2, 1:2) - route(idx, 1:2);
    if norm(d) < 1e-9
        yaw = pi / 2;
    else
        yaw = atan2(d(2), d(1));
    end
end

function v = limitVector(v, limit)
    n = norm(v);
    if n > limit
        v = v * (limit / n);
    end
end

function clearance = minObstacleClearance(pos, s)
    clearance = inf;
    for i = 1:size(s.obstacles, 1)
        obs = s.obstacles(i, :);
        if pos(3) >= obs(3) && pos(3) <= obs(4)
            c = hypot(pos(1) - obs(1), pos(2) - obs(2)) - ...
                obs(5) - s.droneRadius;
            clearance = min(clearance, c);
        end
    end
end

function writeDynamicCsv(csvFile, data)
    stateNames = strcat('true_', data.outputNames);
    names = [{'t'}, data.inputNames, data.outputNames, stateNames];
    values = [data.t(:), data.U', data.Y', data.X'];
    tbl = array2table(values, 'VariableNames', names);
    writetable(tbl, csvFile);
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
