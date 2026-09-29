function result = final_run_deepc_tracking(common, scenarioId, varargin)
%FINAL_RUN_DEEPC_TRACKING Run DeePC on the fixed final-comparison reference.

    p = inputParser;
    addParameter(p, 'UseObstacles', false, @islogical);
    addParameter(p, 'ObstacleOptions', struct(), @isstruct);
    parse(p, varargin{:});
    opts = p.Results;

    scenario = final_apply_obstacle_scenario(common, scenarioId);
    common = selectScenarioReference(common, scenarioId);
    config = common.config;
    config.scenario = scenario;

    state = common.initialState(:);
    m = config.m;
    pOut = config.p;
    Tini = config.T_ini;
    Tf = config.T_f;
    nSteps = common.numSteps;

    y0 = state(scenario.outputIdx);
    uZero = (zeros(m, 1) - config.uOffset) ./ config.uScale;
    yNorm = (y0 - config.yOffset) ./ config.yScale;
    uHist = repmat(uZero, 1, Tini);
    yHist = repmat(yNorm, 1, Tini);

    log = initLog(nSteps, m, pOut);
    prevPredictedHorizon = [];

    for k = 1:nSteps
        [r, refHorizon] = buildReferenceHorizon(common, config, k);

        if opts.UseObstacles
            if isempty(prevPredictedHorizon)
                pNom = refHorizon;
            else
                pNom = [prevPredictedHorizon(:, 2:end), refHorizon(:, end)];
            end
            if isfield(opts.ObstacleOptions, 'MaxIter')
                [uOpt, yPred, info] = solveIterative(config, uHist(:), yHist(:), r, refHorizon, ...
                    prevPredictedHorizon, opts.ObstacleOptions);
            else
                [uOpt, yPred, info] = branch_solve_obstacle_deepc(config, uHist(:), yHist(:), ...
                    r, pNom, opts.ObstacleOptions);
                info.iterationsUsed = 1;
                info.convergenceResidual = 0;
            end
            prevPredictedHorizon = info.predictedPositions;
        else
            [uOpt, yPred, info] = deepcAlgorithm(config.model, uHist(:), yHist(:), r, config.solverOpts);
            info.slack = [];
            info.iterationsUsed = 1;
            info.convergenceResidual = 0;
            info.constraintCount = 0;
            info.predictedPositions = predictionPositions(config, yPred);
        end

        if info.success && ~isempty(uOpt) && all(isfinite(uOpt))
            uNorm = uOpt(1:m);
        else
            uNorm = uZero;
        end

        uCmd = uNorm .* config.uScale + config.uOffset;
        uCmd = max(config.plantCfg.uMin, min(config.plantCfg.uMax, uCmd));
        [state, yFull, uApplied] = simulateLabyrinthDroneStep(state, uCmd, config.plantCfg);

        yPhys = yFull(scenario.outputIdx);
        yNorm = (yPhys - config.yOffset) ./ config.yScale;
        uNormApplied = (uApplied - config.uOffset) ./ config.uScale;

        uHist = [uHist(:, 2:end), uNormApplied];
        yHist = [yHist(:, 2:end), yNorm];

        log.time(k) = common.t_ref(k);
        log.pos(:, k) = yPhys(1:3);
        log.vel(:, k) = yPhys(4:6);
        log.ref(:, k) = common.p_ref(:, k);
        log.u(:, k) = uApplied;
        log.success(k) = info.success;
        log.solveTime(k) = info.solveTime;
        log.err(k) = norm(yPhys(1:3) - common.p_ref(:, k));
        log.slackMean(k) = safeMean(info.slack);
        log.slackMax(k) = safeMax(info.slack);
        log.iterationCount(k) = getInfoValue(info, 'iterationsUsed', 1);
        log.convergenceResidual(k) = getInfoValue(info, 'convergenceResidual', 0);
        log.constraintCount(k) = getInfoValue(info, 'constraintCount', 0);
        log.predHorizon{k} = getInfoValue(info, 'predictedPositions', zeros(3, Tf));
    end

    [h, safetyDetail] = final_compute_safety_margin(log.pos', scenario, ...
        'UseObstacles', opts.UseObstacles, 'UseWalls', false);
    motorEffort = final_generate_motor_efforts_mpc(log.u, config.plantCfg, 'Controller', 'DeePC');

    result = struct();
    result.scenarioId = upper(char(scenarioId));
    result.controller = 'DeePC';
    result.usesObstacles = opts.UseObstacles;
    result.scenario = scenario;
    result.reference = struct('time', common.t_ref, 'position', common.p_ref, 'velocity', common.v_ref);
    result.initialState = common.initialState;
    result.log = log;
    result.safetyMargin = h;
    result.safetyDetail = safetyDetail;
    result.motorEffort = motorEffort;
    result.configSummary = struct('T_ini', config.T_ini, 'T_f', config.T_f, ...
        'lambda_g', config.params.lambda_g, 'lambda_y', config.params.lambda_y);
    result.obstacleOptions = opts.ObstacleOptions;
    result.metrics = final_compute_metrics(result);
end

function [uOpt, yPred, info] = solveIterative(config, uIni, yIni, r, refHorizon, prevPredictedHorizon, obstacleOptions)
    maxIter = getOption(obstacleOptions, 'MaxIter', 4);
    tolerance = getOption(obstacleOptions, 'Tolerance', 1e-3);

    if isempty(prevPredictedHorizon)
        pNom = refHorizon;
    else
        pNom = [prevPredictedHorizon(:, 2:end), refHorizon(:, end)];
    end

    iterates = cell(1, maxIter);
    residual = 0;
    for iter = 1:maxIter
        [uOpt, yPred, info] = branch_solve_obstacle_deepc(config, uIni, yIni, r, pNom, obstacleOptions);
        iterates{iter} = info.predictedPositions;
        if ~info.success
            break;
        end
        pPred = info.predictedPositions;
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

function [r, refHorizon] = buildReferenceHorizon(common, config, idx)
    rMat = zeros(config.p, config.T_f);
    refHorizon = zeros(3, config.T_f);
    for j = 1:config.T_f
        idxJ = min(idx + j - 1, common.numSteps);
        posRef = common.p_ref(:, idxJ);
        velRef = common.v_ref(:, idxJ);
        yRef = [posRef; velRef];
        rMat(:, j) = (yRef - config.yOffset) ./ config.yScale;
        refHorizon(:, j) = posRef;
    end
    r = rMat(:);
end

function positions = predictionPositions(config, yPred)
    if isempty(yPred)
        positions = zeros(3, config.T_f);
        return;
    end
    positions = zeros(3, config.T_f);
    for j = 1:config.T_f
        rows = (j - 1) * config.p + (1:3);
        positions(:, j) = yPred(rows) .* config.positionScale + config.positionOffset;
    end
end

function log = initLog(n, m, pOut)
    log = struct();
    log.time = zeros(1, n);
    log.pos = zeros(3, n);
    log.vel = zeros(3, n);
    log.ref = zeros(3, n);
    log.u = zeros(m, n);
    log.success = false(1, n);
    log.solveTime = zeros(1, n);
    log.err = zeros(1, n);
    log.slackMean = zeros(1, n);
    log.slackMax = zeros(1, n);
    log.iterationCount = zeros(1, n);
    log.convergenceResidual = zeros(1, n);
    log.constraintCount = zeros(1, n);
    log.predHorizon = cell(1, n);
    log.pred = zeros(pOut, n);
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
