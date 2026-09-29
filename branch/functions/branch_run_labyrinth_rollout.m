function result = branch_run_labyrinth_rollout(config, options)
%BRANCH_RUN_LABYRINTH_ROLLOUT Run a branch-side DeePC rollout on the labyrinth scenario.

    arguments
        config struct
        options.Label char = 'branch_rollout'
        options.ControllerMode char = 'baseline'
        options.MaxSteps (1,1) double = 280
        options.ValidateOriginal (1,1) logical = false
        options.ObstacleOptions struct = struct()
    end

    scenario = config.scenario;
    state = config.initialState;

    m = config.m;
    p = config.p;
    T_ini = config.T_ini;
    T_f = config.T_f;
    dt = config.dt;

    y0 = selectOutput(state, scenario);
    uZero = (zeros(m, 1) - config.uOffset) ./ config.uScale;
    yNorm = (y0 - config.yOffset) ./ config.yScale;

    uHist = repmat(uZero, 1, T_ini);
    yHist = repmat(yNorm, 1, T_ini);

    log = initLog(options.MaxSteps, m, p);
    pathIdx = 1;
    prevPredictedHorizon = [];

    for k = 1:options.MaxSteps
        pos = state(1:3)';
        pathIdx = advancePathIndex(config.path, pathIdx, pos, 0.26);

        [r, refHorizon] = buildReferenceHorizon(config, pathIdx);

        switch lower(options.ControllerMode)
            case 'baseline'
                [uOpt, yPred, info] = deepcAlgorithm( ...
                    config.model, uHist(:), yHist(:), r, config.solverOpts);
                info.slack = [];
                info.iterationsUsed = 1;
                info.convergenceResidual = 0;
                info.constraintCount = 0;
                info.predictedPositions = predictionPositions(config, yPred);

            case 'soft'
                if isempty(prevPredictedHorizon)
                    pNom = refHorizon;
                else
                    pNom = [prevPredictedHorizon(:, 2:end), refHorizon(:, end)];
                end
                [uOpt, yPred, info] = branch_solve_obstacle_deepc( ...
                    config, uHist(:), yHist(:), r, pNom, options.ObstacleOptions);
                info.iterationsUsed = 1;
                prevPredictedHorizon = info.predictedPositions;

            case 'iterative'
                [uOpt, yPred, info] = solveIterativeMode( ...
                    config, uHist(:), yHist(:), r, refHorizon, prevPredictedHorizon, options.ObstacleOptions);
                prevPredictedHorizon = info.predictedPositions;

            otherwise
                error('branch:badControllerMode', 'Unknown controller mode: %s', options.ControllerMode);
        end

        if info.success && ~isempty(uOpt) && all(isfinite(uOpt))
            uNorm = uOpt(1:m);
        else
            uNorm = uZero;
        end

        uCmd = uNorm .* config.uScale + config.uOffset;
        uCmd = max(config.plantCfg.uMin, min(config.plantCfg.uMax, uCmd));
        [state, yFull, uApplied] = simulateLabyrinthDroneStep(state, uCmd, config.plantCfg);

        yPhys = selectOutput(yFull, scenario);
        yNorm = (yPhys - config.yOffset) ./ config.yScale;
        uNormApplied = (uApplied - config.uOffset) ./ config.uScale;

        uHist = [uHist(:, 2:end), uNormApplied];
        yHist = [yHist(:, 2:end), yNorm];

        refPhys = config.path(pathIdx, :)';
        log.time(k) = (k - 1) * dt;
        log.pos(:, k) = yPhys(1:3);
        log.vel(:, k) = yPhys(4:6);
        log.ref(:, k) = refPhys;
        log.u(:, k) = uApplied;
        log.success(k) = info.success;
        log.pred(:, k) = denormalizePrediction(yPred, config.yOffset, config.yScale, p);
        log.pathIdx(k) = pathIdx;
        log.err(k) = norm(yPhys(1:3) - refPhys);
        log.solveTime(k) = info.solveTime;
        log.slackMean(k) = meanOrZero(info.slack);
        log.slackMax(k) = maxOrZero(info.slack);
        log.iterationCount(k) = info.iterationsUsed;
        log.convergenceResidual(k) = info.convergenceResidual;
        log.constraintCount(k) = info.constraintCount;
        log.predHorizon{k} = info.predictedPositions;
        if isfield(info, 'iterates')
            log.iterates{k} = info.iterates;
        else
            log.iterates{k} = {};
        end

        if mod(k, 25) == 0 || k == 1
            fprintf(['%s | t = %.1f s | pos = [%.2f %.2f %.2f] | ' ...
                'ref = [%.2f %.2f %.2f] | qp = %d | slack = %.4f\n'], ...
                options.Label, log.time(k), yPhys(1), yPhys(2), yPhys(3), ...
                refPhys(1), refPhys(2), refPhys(3), info.success, log.slackMax(k));
        end

        if pathIdx >= size(config.path, 1) && norm(yPhys(1:3) - scenario.B(:)) < 0.35
            log = trimLog(log, k);
            break;
        end
    end

    if log.time(end) == 0 && numel(log.time) > 1
        lastIdx = find(log.time > 0, 1, 'last');
        if isempty(lastIdx)
            lastIdx = 1;
        end
        log = trimLog(log, lastIdx);
    end

    metrics = branch_compute_metrics(log, scenario);
    [safetyMargin, safetyDetail] = branch_compute_safety_margin(log.pos', scenario);

    result = struct();
    result.label = options.Label;
    result.controllerMode = options.ControllerMode;
    result.configSummary = summarizeConfig(config);
    result.scenario = scenario;
    result.path = config.path;
    result.pathSource = config.pathSource;
    result.meta = config.meta;
    result.modelDims = config.model.dims;
    result.log = log;
    result.safetyMargin = safetyMargin;
    result.safetyDetail = safetyDetail;
    result.metrics = metrics;

    if options.ValidateOriginal
        branch_validate_original_result(result);
    end
end

function [uOpt, yPred, info] = solveIterativeMode(config, uIni, yIni, r, refHorizon, prevPredictedHorizon, obstacleOptions)
    maxIter = getOption(obstacleOptions, 'MaxIter', 4);
    tolerance = getOption(obstacleOptions, 'Tolerance', 1e-3);

    if isempty(prevPredictedHorizon)
        pNom = refHorizon;
    else
        pNom = [prevPredictedHorizon(:, 2:end), refHorizon(:, end)];
    end

    iterates = cell(1, maxIter);
    residual = 0;
    lastInfo = struct();
    uOpt = [];
    yPred = [];

    for iter = 1:maxIter
        [uOpt, yPred, lastInfo] = branch_solve_obstacle_deepc( ...
            config, uIni, yIni, r, pNom, obstacleOptions);

        iterates{iter} = lastInfo.predictedPositions;
        if ~lastInfo.success
            break;
        end

        pPred = lastInfo.predictedPositions;
        residual = norm(pPred(:) - pNom(:)) / max(1, norm(pNom(:)));
        pNom = pPred;

        if residual < tolerance
            iterates = iterates(1:iter);
            break;
        end
    end

    if isempty(lastInfo)
        lastInfo = struct();
    end
    lastInfo.iterates = iterates;
    lastInfo.iterationsUsed = numel(iterates);
    lastInfo.convergenceResidual = residual;
    info = lastInfo;
end

function [r, refHorizon] = buildReferenceHorizon(config, idx)
    p = config.p;
    T_f = config.T_f;
    rMat = zeros(p, T_f);
    refHorizon = zeros(3, T_f);
    for j = 1:T_f
        idxJ = min(idx + j - 1, size(config.path, 1));
        posRef = config.path(idxJ, :)';
        velRef = pathVelocity(config.path, idxJ, config.dt);
        yRef = [posRef; velRef];
        rMat(:, j) = (yRef - config.yOffset) ./ config.yScale;
        refHorizon(:, j) = posRef;
    end
    r = rMat(:);
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

function vel = pathVelocity(path, idx, dt)
    if idx >= size(path, 1)
        vel = zeros(3, 1);
        return;
    end
    step = path(min(idx + 1, size(path, 1)), :) - path(idx, :);
    vel = step(:) / max(dt, 1e-9);
    speed = norm(vel);
    if speed > 1.05
        vel = vel * (1.05 / speed);
    end
end

function yPredPhys = denormalizePrediction(yPred, yOffset, yScale, p)
    if isempty(yPred)
        yPredPhys = zeros(p, 1);
    else
        yPredPhys = yPred(1:p) .* yScale + yOffset;
    end
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

function y = selectOutput(stateOrOutput, scenario)
    y = stateOrOutput(scenario.outputIdx);
end

function log = initLog(n, m, p)
    log = struct();
    log.time = zeros(1, n);
    log.pos = zeros(3, n);
    log.vel = zeros(3, n);
    log.ref = zeros(3, n);
    log.u = zeros(m, n);
    log.success = false(1, n);
    log.pred = zeros(p, n);
    log.pathIdx = zeros(1, n);
    log.err = zeros(1, n);
    log.solveTime = zeros(1, n);
    log.slackMean = zeros(1, n);
    log.slackMax = zeros(1, n);
    log.iterationCount = zeros(1, n);
    log.convergenceResidual = zeros(1, n);
    log.constraintCount = zeros(1, n);
    log.predHorizon = cell(1, n);
    log.iterates = cell(1, n);
end

function log = trimLog(log, n)
    fields = fieldnames(log);
    for i = 1:numel(fields)
        value = log.(fields{i});
        if iscell(value)
            log.(fields{i}) = value(1:n);
        elseif isvector(value)
            log.(fields{i}) = value(1:n);
        else
            log.(fields{i}) = value(:, 1:n);
        end
    end
end

function summary = summarizeConfig(config)
    summary = struct();
    summary.dataSource = config.dataSource;
    summary.dataFile = config.dataFile;
    summary.T_ini = config.T_ini;
    summary.T_f = config.T_f;
    summary.lambda_y = config.params.lambda_y;
    summary.lambda_g = config.params.lambda_g;
    summary.pathSource = config.pathSource;
    summary.pathSpacing = config.pathSpacing;
end

function value = meanOrZero(x)
    if isempty(x)
        value = 0;
    else
        value = mean(x);
    end
end

function value = maxOrZero(x)
    if isempty(x)
        value = 0;
    else
        value = max(x);
    end
end

function value = getOption(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
