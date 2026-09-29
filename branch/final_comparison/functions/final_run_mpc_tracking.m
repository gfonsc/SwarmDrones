function result = final_run_mpc_tracking(common, scenarioId, varargin)
%FINAL_RUN_MPC_TRACKING Run MPC on the fixed final-comparison reference.

    p = inputParser;
    addParameter(p, 'UseObstacles', false, @islogical);
    addParameter(p, 'ObstacleOptions', struct(), @isstruct);
    parse(p, varargin{:});
    opts = p.Results;

    scenario = final_apply_obstacle_scenario(common, scenarioId);
    common = selectScenarioReference(common, scenarioId);

    scenarioData = struct();
    scenarioData.scenario = scenario;
    scenarioData.path = common.p_ref';
    scenarioData.pathSource = common.referenceSource;
    scenarioData.dt = common.dt;
    scenarioData.initialState = common.initialState;
    scenarioData.plantCfg = common.plantCfg;
    scenarioData.maxSteps = common.numSteps;
    scenarioData.inputNames = scenario.inputNames;
    scenarioData.outputNames = scenario.outputNames;

    model = mpc_adapt_tcc_model_to_labyrinth(scenarioData);
    params = model.defaultParams;
    params.N = common.config.T_f;

    state = common.initialState(:);
    prevInput = zeros(4, 1);
    prevPredicted = [];
    log = initLog(common.numSteps);

    for k = 1:common.numSteps
        [ref, refHorizon] = buildReferenceHorizon(common, k, params.N, state(7));
        params.u_prev = prevInput;

        if opts.UseObstacles
            if isempty(prevPredicted)
                pNom = refHorizon;
            else
                pNom = [prevPredicted(:, 2:end), refHorizon(:, end)];
            end
            if isfield(opts.ObstacleOptions, 'MaxIter')
                [uSeq, xPred, info] = solveIterative(model, state, ref, scenario, params, ...
                    refHorizon, prevPredicted, opts.ObstacleOptions);
            else
                obsOpts = opts.ObstacleOptions;
                obsOpts.NominalTrajectory = pNom;
                [uSeq, xPred, info] = mpc_solve_obstacle_aware(model, state, ref, scenario, params, obsOpts);
                info.iterationsUsed = 1;
                info.convergenceResidual = 0;
            end
        else
            [uSeq, xPred, info] = mpc_solve_tracking(model, state, ref, params);
            info.slack = [];
            info.iterationsUsed = 1;
            info.convergenceResidual = 0;
            info.constraintCount = 0;
        end

        if info.success
            uCmd = uSeq(:, 1);
        else
            uCmd = zeros(4, 1);
        end
        prevInput = uCmd;

        [state, yFull, uApplied] = simulateLabyrinthDroneStep(state, uCmd, scenarioData.plantCfg);
        prevPredicted = xPred(1:3, 2:end);

        log.time(k) = common.t_ref(k);
        log.state(:, k) = state;
        log.pos(:, k) = yFull(1:3);
        log.vel(:, k) = yFull(4:6);
        log.ref(:, k) = common.p_ref(:, k);
        log.u(:, k) = uApplied;
        log.success(k) = info.success;
        log.solveTime(k) = info.solveTime;
        log.err(k) = norm(yFull(1:3) - common.p_ref(:, k));
        log.slackMean(k) = safeMean(info.slack);
        log.slackMax(k) = safeMax(info.slack);
        log.iterationCount(k) = getInfoValue(info, 'iterationsUsed', 1);
        log.convergenceResidual(k) = getInfoValue(info, 'convergenceResidual', 0);
        log.constraintCount(k) = getInfoValue(info, 'constraintCount', 0);
        log.predHorizon{k} = xPred(1:3, 2:end);
    end

    [h, safetyDetail] = final_compute_safety_margin(log.pos', scenario, ...
        'UseObstacles', opts.UseObstacles, 'UseWalls', false);
    motorEffort = final_generate_motor_efforts_mpc(log.u, common.plantCfg, 'Controller', 'MPC');

    result = struct();
    result.scenarioId = upper(char(scenarioId));
    result.controller = 'MPC';
    result.usesObstacles = opts.UseObstacles;
    result.scenario = scenario;
    result.reference = struct('time', common.t_ref, 'position', common.p_ref, 'velocity', common.v_ref);
    result.initialState = common.initialState;
    result.model = model;
    result.controllerParams = params;
    result.log = log;
    result.safetyMargin = h;
    result.safetyDetail = safetyDetail;
    result.motorEffort = motorEffort;
    result.obstacleOptions = opts.ObstacleOptions;
    result.metrics = final_compute_metrics(result);
end

function [uSeq, xPred, info] = solveIterative(model, state, ref, scenario, params, refHorizon, prevPredicted, obstacleOptions)
    maxIter = getOption(obstacleOptions, 'MaxIter', 4);
    tolerance = getOption(obstacleOptions, 'Tolerance', 1e-3);
    if isempty(prevPredicted)
        pNom = refHorizon;
    else
        pNom = [prevPredicted(:, 2:end), refHorizon(:, end)];
    end

    iterates = cell(1, maxIter);
    residual = 0;
    for iter = 1:maxIter
        obsOpts = obstacleOptions;
        obsOpts.NominalTrajectory = pNom;
        [uSeq, xPred, info] = mpc_solve_obstacle_aware(model, state, ref, scenario, params, obsOpts);
        iterates{iter} = xPred(1:3, 2:end);
        if ~info.success
            break;
        end
        pPred = xPred(1:3, 2:end);
        residual = norm(pPred(:) - pNom(:)) / max(1, norm(pNom(:)));
        pNom = pPred;
        if residual < tolerance
            iterates = iterates(1:iter);
            break;
        end
    end
    info.iterates = iterates;
    info.iterationsUsed = numel(iterates);
    info.convergenceResidual = residual;
end

function [ref, refHorizon] = buildReferenceHorizon(common, idx, N, yawRef)
    ref.pos = zeros(3, N);
    ref.vel = zeros(3, N);
    ref.yaw = yawRef * ones(1, N);
    ref.yaw_rate = zeros(1, N);
    refHorizon = zeros(3, N);
    for j = 1:N
        idxJ = min(idx + j - 1, common.numSteps);
        ref.pos(:, j) = common.p_ref(:, idxJ);
        ref.vel(:, j) = common.v_ref(:, idxJ);
        refHorizon(:, j) = ref.pos(:, j);
    end
end

function log = initLog(n)
    log = struct();
    log.time = zeros(1, n);
    log.state = zeros(8, n);
    log.pos = zeros(3, n);
    log.vel = zeros(3, n);
    log.ref = zeros(3, n);
    log.u = zeros(4, n);
    log.success = false(1, n);
    log.solveTime = zeros(1, n);
    log.err = zeros(1, n);
    log.slackMean = zeros(1, n);
    log.slackMax = zeros(1, n);
    log.iterationCount = zeros(1, n);
    log.convergenceResidual = zeros(1, n);
    log.constraintCount = zeros(1, n);
    log.predHorizon = cell(1, n);
end

function value = safeMean(x)
    if isempty(x)
        value = 0;
    else
        value = mean(x);
    end
end

function value = safeMax(x)
    if isempty(x)
        value = 0;
    else
        value = max(x);
    end
end

function value = getInfoValue(info, fieldName, defaultValue)
    if isfield(info, fieldName)
        value = info.(fieldName);
    else
        value = defaultValue;
    end
end

function value = getOption(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end

function common = selectScenarioReference(common, scenarioId)
    switch upper(char(scenarioId))
        case 'A'
            if isfield(common, 'referenceA')
                ref = common.referenceA;
            else
                return;
            end
        case 'B'
            if isfield(common, 'referenceB')
                ref = common.referenceB;
            else
                return;
            end
        otherwise
            return;
    end
    common.t_ref = ref.time;
    common.p_ref = ref.position;
    common.v_ref = ref.velocity;
    common.numSteps = numel(ref.time);
    common.simulationTime = ref.time(end);
    common.startPoint = ref.position(:, 1);
    common.finishPoint = ref.position(:, end);
    common.referenceSource = ref.source;
    common.yaw_ref = common.config.initialState(7) * ones(1, common.numSteps);
end
