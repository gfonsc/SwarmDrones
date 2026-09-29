function dataFile = generateLabyrinthDeePCDatabase(varargin)
% GENERATELABYRINTHDEEPCDATABASE Create offline I/O data for DeePC labyrinth tests.
%
% The database is trajectory-level: inputs are velocity/yaw-rate commands
% and outputs are position plus velocity inside the labyrinth envelope.

    p = inputParser;
    addParameter(p, 'OutputDir', fullfile(fileparts(mfilename('fullpath')), 'labyrinth'), @ischar);
    addParameter(p, 'NumSamples', 360, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Seed', 7, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'WriteCsv', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    if ~exist(opts.OutputDir, 'dir')
        mkdir(opts.OutputDir);
    end

    baseDir = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(baseDir, 'models'));

    scenario = labyrinthScenarioConfig();
    rng(opts.Seed);

    N = round(opts.NumSamples);
    cfg = scenario.model;
    outputIdx = scenario.outputIdx;

    U = zeros(numel(cfg.uMin), N);
    Y = zeros(numel(outputIdx), N);
    X = zeros(8, N);
    clearance = zeros(1, N);

    state = [scenario.A(:); 0; 0; 0; pi/2; 0];
    route = buildExcitationRoute(scenario);
    targetIdx = 1;
    target = route(targetIdx, :)';

    ditherPhase = 2 * pi * rand(4, 1);
    for k = 1:N
        if norm(target - state(1:3)) < 0.35 || mod(k, 42) == 0
            targetIdx = mod(targetIdx, size(route, 1)) + 1;
            target = route(targetIdx, :)';
        end

        err = target - state(1:3);
        uTrack = [0.95 * err(1); 0.95 * err(2); 0.90 * err(3); 0.0];

        dither = [
            0.23 * sin(0.31 * k + ditherPhase(1)) + 0.10 * sin(0.071 * k);
            0.21 * sin(0.27 * k + ditherPhase(2)) + 0.08 * cos(0.053 * k);
            0.12 * sin(0.19 * k + ditherPhase(3));
            0.35 * sin(0.23 * k + ditherPhase(4))
        ];
        randomHold = 0.055 * randn(4, 1);

        uCmd = uTrack + dither + randomHold;
        [state, yFull, uApplied] = simulateLabyrinthDroneStep(state, uCmd, cfg);

        yNoise = [
            0.0015 * randn(3, 1);
            0.0060 * randn(3, 1)
        ];

        U(:, k) = uApplied;
        Y(:, k) = yFull(outputIdx) + yNoise;
        X(:, k) = state;
        clearance(k) = minObstacleClearance(state(1:3)', scenario);
    end

    t = (0:N - 1) * cfg.dt;

    data = struct();
    data.U = U;
    data.Y = Y;
    data.X = X;
    data.t = t;
    data.scenario = scenario;
    data.outputIdx = outputIdx;
    data.inputNames = scenario.inputNames;
    data.outputNames = scenario.outputNames;
    data.meta = struct( ...
        'description', 'Labyrinth DeePC trajectory-level velocity-command database', ...
        'inputModel', '[vx_cmd; vy_cmd; vz_cmd; yaw_rate_cmd]', ...
        'outputModel', '[x; y; z; vx; vy; vz]', ...
        'numSamples', N, ...
        'dt', cfg.dt, ...
        'seed', opts.Seed, ...
        'minObstacleClearance', min(clearance));

    dataFile = fullfile(opts.OutputDir, 'labyrinth_deepc_data.mat');
    save(dataFile, 'data');

    if opts.WriteCsv
        csvFile = fullfile(opts.OutputDir, 'labyrinth_deepc_data.csv');
        writeLabyrinthCsv(csvFile, data);
    end

    fprintf('Generated labyrinth DeePC database:\n');
    fprintf('  MAT: %s\n', dataFile);
    if opts.WriteCsv
        fprintf('  CSV: %s\n', csvFile);
    end
    fprintf('  Samples: %d | dt: %.3f s | min clearance in excitation: %.3f m\n', ...
        N, cfg.dt, data.meta.minObstacleClearance);
end

function route = buildExcitationRoute(s)
    if isfield(s, 'trainingSetpoints') && ~isempty(s.trainingSetpoints)
        route = s.trainingSetpoints;
    else
        route = s.setpoints;
    end
    route(1, :) = [s.A(1), s.A(2), s.zHover];

    midOffsets = [
        0.18,  0.00,  0.00;
       -0.18,  0.00,  0.00;
        0.00,  0.18,  0.10;
        0.00, -0.18, -0.10
    ];

    expanded = zeros(0, 3);
    for i = 1:size(route, 1)
        expanded(end + 1, :) = route(i, :); %#ok<AGROW>
        if i > 2 && i < size(route, 1) - 1
            off = midOffsets(mod(i - 1, size(midOffsets, 1)) + 1, :);
            p = route(i, :) + off;
            p(1) = min(max(p(1), s.bounds.x(1) + 0.15), s.bounds.x(2) - 0.15);
            p(2) = min(max(p(2), s.bounds.y(1) + 0.15), s.bounds.y(2) - 0.15);
            p(3) = min(max(p(3), s.bounds.z(1) + 0.20), s.bounds.z(2) - 0.12);
            expanded(end + 1, :) = p; %#ok<AGROW>
        end
    end

    route = [expanded; flipud(expanded); expanded];
end

function writeLabyrinthCsv(csvFile, data)
    names = [{'t'}, data.inputNames, data.outputNames, ...
        {'state_x', 'state_y', 'state_z', 'state_vx', 'state_vy', 'state_vz', ...
         'state_yaw', 'state_yaw_rate'}];
    values = [data.t(:), data.U', data.Y', data.X'];
    tbl = array2table(values, 'VariableNames', names);
    writetable(tbl, csvFile);
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
