function common = final_load_common_reference(varargin)
%FINAL_LOAD_COMMON_REFERENCE Build one fixed reference for DeePC and MPC.

    p = inputParser;
    addParameter(p, 'NumSteps', 420, @(x) isnumeric(x) && isscalar(x) && x >= 20);
    addParameter(p, 'PathSpacing', 0.16, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});
    opts = p.Results;

    paths = run_00_setup_final_comparison_paths();

    evalc(['cfg = branch_extract_demo_config(' ...
        '''RegenerateIfMissing'', true, ' ...
        '''NumSamples'', 360, ' ...
        '''Seed'', 11, ' ...
        '''TIni'', 4, ' ...
        '''TF'', 8, ' ...
        '''LambdaY'', 700, ' ...
        '''LambdaG'', 12, ' ...
        '''PathSpacing'', opts.PathSpacing, ' ...
        '''UseTrainingSetpoints'', true, ' ...
        '''SolverName'', ''auto'', ' ...
        '''SolverVerbose'', false);']);

    dt = cfg.scenario.dt;
    pathA = cfg.scenario.setpoints;
    if isfield(cfg.scenario, 'trainingSetpoints') && ~isempty(cfg.scenario.trainingSetpoints)
        pathB = cfg.scenario.trainingSetpoints;
    else
        pathB = cfg.scenario.setpoints;
    end
    pRefA = resamplePathToFixedSteps(pathA, opts.NumSteps);
    pRefB = resamplePathToFixedSteps(pathB, opts.NumSteps);
    tRef = (0:opts.NumSteps - 1) * dt;
    vRefA = referenceVelocity(pRefA, dt);
    vRefB = referenceVelocity(pRefB, dt);

    scenarioA = cfg.scenario;
    scenarioA.name = 'Scenario A - setpoint tracking with labyrinth only';
    scenarioA.obstacles = zeros(0, size(cfg.scenario.obstacles, 2));
    scenarioA.activeObstacleDescription = 'No cylindrical obstacles; labyrinth walls are preserved for visualization.';

    scenarioB = cfg.scenario;
    scenarioB.name = 'Scenario B - setpoint tracking with labyrinth and cylindrical obstacles';
    scenarioB.activeObstacleDescription = 'Original branch cylindrical obstacles active.';

    common = struct();
    common.paths = paths;
    common.config = cfg;
    common.dt = dt;
    common.t_ref = tRef;
    common.p_ref = pRefA;
    common.v_ref = vRefA;
    common.referenceA = struct('time', tRef, 'position', pRefA, 'velocity', vRefA, ...
        'source', 'Fixed resampling of original labyrinth mission setpoints');
    common.referenceB = struct('time', tRef, 'position', pRefB, 'velocity', vRefB, ...
        'source', 'Fixed resampling of original labyrinth training setpoints with obstacle/labyrinth deviations');
    common.numSteps = opts.NumSteps;
    common.simulationTime = tRef(end);
    common.startPoint = pRefA(:, 1);
    common.finishPoint = pRefA(:, end);
    common.nominalSetpoints = cfg.scenario.setpoints';
    common.initialState = cfg.initialState(:);
    common.plantCfg = cfg.plantCfg;
    common.referenceSource = common.referenceA.source;
    common.referenceSourceA = common.referenceA.source;
    common.referenceSourceB = common.referenceB.source;
    common.yaw_ref = cfg.initialState(7) * ones(1, opts.NumSteps);
    common.scenarioA = scenarioA;
    common.scenarioB = scenarioB;
    common.obstaclesScenarioB = scenarioB.obstacles;
    common.safetyMetricUsesWalls = false;
    common.safetyMetricUsesObstacles = true;
end

function pRef = resamplePathToFixedSteps(path, nSteps)
    d = diff(path, 1, 1);
    segLen = sqrt(sum(d.^2, 2));
    s = [0; cumsum(segLen)];
    keep = [true; diff(s) > 1e-10];
    s = s(keep);
    path = path(keep, :);
    sampleS = linspace(0, s(end), nSteps);
    pRef = [
        interp1(s, path(:, 1), sampleS, 'linear')
        interp1(s, path(:, 2), sampleS, 'linear')
        interp1(s, path(:, 3), sampleS, 'linear')
    ];
end

function vRef = referenceVelocity(pRef, dt)
    vRef = zeros(size(pRef));
    if size(pRef, 2) <= 1
        return;
    end
    vRef(:, 1:end-1) = diff(pRef, 1, 2) / max(dt, 1e-9);
    vRef(:, end) = vRef(:, end-1);
    for k = 1:size(vRef, 2)
        speed = norm(vRef(:, k));
        if speed > 1.05
            vRef(:, k) = vRef(:, k) * (1.05 / speed);
        end
    end
end
