function config = branch_extract_demo_config(varargin)
%BRANCH_EXTRACT_DEMO_CONFIG Build a branch-side config from the original labyrinth demo.

    p = inputParser;
    addParameter(p, 'RegenerateIfMissing', true, @islogical);
    addParameter(p, 'NumSamples', 360, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Seed', 11, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'TIni', 4, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'TF', 8, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'LambdaY', 700, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'LambdaG', 12, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'EpsReg', 1e-7, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Q', diag([850, 850, 980, 8, 8, 10]), @isnumeric);
    addParameter(p, 'R', diag([0.018, 0.018, 0.030, 0.10]), @isnumeric);
    addParameter(p, 'PathSpacing', 0.16, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'UseTrainingSetpoints', true, @islogical);
    addParameter(p, 'SolverName', 'auto', @ischar);
    addParameter(p, 'SolverVerbose', false, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    paths = run_00_setup_branch_paths();

    originalDataFile = fullfile(paths.databaseDir, 'labyrinth', 'labyrinth_deepc_data.mat');
    dataSource = 'original_database_mat';

    if isfile(originalDataFile)
        S = load(originalDataFile, 'data');
        data = S.data;
        dataFile = originalDataFile;
    else
        if ~opts.RegenerateIfMissing
            error('branch:missingDatabase', ...
                'Original labyrinth database is missing and regeneration is disabled.');
        end
        if ~exist(paths.generatedDatabaseDir, 'dir')
            mkdir(paths.generatedDatabaseDir);
        end
        dataFile = generateLabyrinthDeePCDatabase( ...
            'OutputDir', paths.generatedDatabaseDir, ...
            'NumSamples', opts.NumSamples, ...
            'Seed', opts.Seed, ...
            'WriteCsv', true);
        S = load(dataFile, 'data');
        data = S.data;
        dataSource = 'branch_generated_database';
    end

    U_train = data.U;
    Y_train = data.Y;
    meta = data;
    scenario = data.scenario;
    plantCfg = scenario.model;

    m = size(U_train, 1);
    pOut = size(Y_train, 1);

    uOffset = mean(U_train, 2);
    yOffset = mean(Y_train, 2);
    uScale = max(max(abs(U_train - uOffset), [], 2), [0.25; 0.25; 0.18; 0.30]);
    yScale = max(max(abs(Y_train - yOffset), [], 2), [0.60; 0.60; 0.30; 0.25; 0.25; 0.18]);

    U_lti = (U_train - uOffset) ./ uScale;
    Y_lti = (Y_train - yOffset) ./ yScale;

    params = struct();
    params.T_ini = opts.TIni;
    params.T_f = opts.TF;
    params.lambda_y = opts.LambdaY;
    params.lambda_g = opts.LambdaG;
    params.qNorm = 2;
    params.epsReg = opts.EpsReg;
    params.Q = opts.Q;
    params.R = opts.R;
    params.u_r = (zeros(m, 1) - uOffset) ./ uScale;
    params.y_r = (zeros(pOut, 1) - yOffset) ./ yScale;
    params.u_min = (plantCfg.uMin - uOffset) ./ uScale;
    params.u_max = (plantCfg.uMax - uOffset) ./ uScale;

    yMinPhys = [scenario.bounds.x(1); scenario.bounds.y(1); scenario.bounds.z(1); ...
        plantCfg.uMin(1:3)];
    yMaxPhys = [scenario.bounds.x(2); scenario.bounds.y(2); scenario.bounds.z(2); ...
        plantCfg.uMax(1:3)];
    params.y_min = (yMinPhys - yOffset) ./ yScale;
    params.y_max = (yMaxPhys - yOffset) ./ yScale;
    params.verbose = true;

    model = deepcSetup(U_lti, Y_lti, params);

    if opts.UseTrainingSetpoints && isfield(scenario, 'trainingSetpoints') && ...
            ~isempty(scenario.trainingSetpoints)
        pathPoints = scenario.trainingSetpoints;
        pathSource = 'trainingSetpoints';
    else
        pathPoints = scenario.setpoints;
        pathSource = 'setpoints';
    end

    densePath = resamplePolyline3D(pathPoints, opts.PathSpacing);

    positionMap = cell(1, opts.TF);
    positionOffset = yOffset(1:3);
    positionScale = yScale(1:3);
    for j = 1:opts.TF
        rows = (j - 1) * pOut + (1:3);
        positionMap{j} = diag(positionScale) * model.Yf(rows, :);
    end

    config = struct();
    config.paths = paths;
    config.dataFile = dataFile;
    config.dataSource = dataSource;
    config.meta = meta;
    config.scenario = scenario;
    config.plantCfg = plantCfg;
    config.U_train = U_train;
    config.Y_train = Y_train;
    config.U_lti = U_lti;
    config.Y_lti = Y_lti;
    config.model = model;
    config.params = params;
    config.solverOpts = struct('solver', opts.SolverName, 'verbose', opts.SolverVerbose);
    config.pathPoints = pathPoints;
    config.pathSource = pathSource;
    config.path = densePath;
    config.pathSpacing = opts.PathSpacing;
    config.m = m;
    config.p = pOut;
    config.T_ini = opts.TIni;
    config.T_f = opts.TF;
    config.dt = scenario.dt;
    config.uOffset = uOffset;
    config.yOffset = yOffset;
    config.uScale = uScale;
    config.yScale = yScale;
    config.positionOffset = positionOffset;
    config.positionScale = positionScale;
    config.positionMap = positionMap;
    config.initialState = [scenario.A(:); 0; 0; 0; pi/2; 0];
    config.originalDemoProfile = struct( ...
        'T_ini', 4, ...
        'T_f', 8, ...
        'lambda_y', 700, ...
        'lambda_g', 12, ...
        'pathSpacing', 0.16);
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
