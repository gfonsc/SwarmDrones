function result = mpc_run_labyrinth_rollout(scenarioData, model, params, varargin)
%MPC_RUN_LABYRINTH_ROLLOUT Run baseline, soft, or iterative MPC on the branch labyrinth scenario.

    p = inputParser;
    addParameter(p, 'Label', 'mpc_rollout', @ischar);
    addParameter(p, 'Mode', 'baseline', @ischar);
    addParameter(p, 'MaxSteps', scenarioData.maxSteps, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'ObstacleOptions', struct(), @isstruct);
    parse(p, varargin{:});
    opts = p.Results;

    scenario = scenarioData.scenario;
    state = scenarioData.initialState(:);
    dt = scenarioData.dt;
    maxSteps = opts.MaxSteps;
    path = scenarioData.path;
    pathIdx = 1;
    prevPredicted = [];
    prevInput = zeros(4, 1);

    log = initLog(maxSteps);

    for k = 1:maxSteps
        pos = state(1:3)';
        pathIdx = advancePathIndex(path, pathIdx, pos, 0.26);
        [ref, refHorizon] = buildReferenceHorizon(path, pathIdx, dt, params.N, state(7));
        params.u_prev = prevInput;

        switch lower(opts.Mode)
            case 'baseline'
                [uSeq, xPred, info] = mpc_solve_tracking(model, state, ref, params);
                info.slack = [];
                info.iterationsUsed = 1;
                info.convergenceResidual = 0;

            case 'soft'
                if isempty(prevPredicted)
                    pNom = refHorizon;
                else
                    pNom = [prevPredicted(:, 2:end), refHorizon(:, end)];
                end
                obsOpts = opts.ObstacleOptions;
                obsOpts.NominalTrajectory = pNom;
                [uSeq, xPred, info] = mpc_solve_obstacle_aware(model, state, ref, scenario, params, obsOpts);
                info.iterationsUsed = 1;
                info.convergenceResidual = 0;

            case 'iterative'
                [uSeq, xPred, info] = solveIterative(model, state, ref, scenario, params, refHorizon, prevPredicted, opts.ObstacleOptions);

            otherwise
                error('mpc:badMode', 'Unknown MPC mode: %s', opts.Mode);
        end

        if info.success
            uCmd = uSeq(:, 1);
        else
            uCmd = zeros(4, 1);
        end
        prevInput = uCmd;

        [state, yFull, uApplied] = simulateLabyrinthDroneStep(state, uCmd, scenarioData.plantCfg);
        prevPredicted = xPred(1:3, 2:end);

        refNow = path(pathIdx, :)';
        log.time(k) = (k - 1) * dt;
        log.state(:, k) = state;
        log.pos(:, k) = yFull(1:3);
        log.vel(:, k) = yFull(4:6);
        log.euler(:, k) = [NaN; NaN; state(7)];
        log.yawRate(k) = state(8);
        log.ref(:, k) = refNow;
        log.u(:, k) = uApplied;
        log.success(k) = info.success;
        log.solveTime(k) = info.solveTime;
        log.err(k) = norm(yFull(1:3) - refNow);
        log.pathIdx(k) = pathIdx;
        log.predHorizon{k} = xPred(1:3, 2:end);
        log.slackMean(k) = safeMean(info.slack);
        log.slackMax(k) = safeMax(info.slack);
        log.iterationCount(k) = info.iterationsUsed;
        log.convergenceResidual(k) = info.convergenceResidual;
        log.constraintCount(k) = info.constraintCount;
        if isfield(info, 'iterates')
            log.iterates{k} = info.iterates;
        else
            log.iterates{k} = {};
        end
        if isfield(info, 'constraintMetadata')
            log.constraintMetadata{k} = info.constraintMetadata;
        else
            log.constraintMetadata{k} = struct();
        end

        if mod(k, 25) == 0 || k == 1
            fprintf(['%s | t = %.1f s | pos = [%.2f %.2f %.2f] | ' ...
                'ref = [%.2f %.2f %.2f] | ok = %d | slack = %.4f\n'], ...
                opts.Label, log.time(k), yFull(1), yFull(2), yFull(3), ...
                refNow(1), refNow(2), refNow(3), info.success, log.slackMax(k));
        end

        if pathIdx >= size(path, 1) && norm(yFull(1:3) - scenario.B(:)) < 0.35
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

    [h, safetyDetail] = mpc_compute_safety_margin(log.pos', scenario);
    log.safetyMargin = h(:)';
    metrics = mpc_compute_article_metrics(log, scenario);

    result = struct();
    result.label = opts.Label;
    result.controllerType = 'MPC';
    result.controllerMode = opts.Mode;
    result.scenario = scenario;
    result.path = path;
    result.pathSource = scenarioData.pathSource;
    result.model = model;
    result.controllerParams = params;
    result.log = log;
    result.safetyMargin = h;
    result.safetyDetail = safetyDetail;
    result.metrics = metrics;
    result.obstacleOptions = opts.ObstacleOptions;
end

function [uSeq, xPred, info] = solveIterative(model, state, ref, scenario, params, refHorizon, prevPredicted, obstacleOptions)
    maxIter = getStructValue(obstacleOptions, 'MaxIter', 4);
    tolerance = getStructValue(obstacleOptions, 'Tolerance', 1e-3);

    if isempty(prevPredicted)
        pNom = refHorizon;
    else
        pNom = [prevPredicted(:, 2:end), refHorizon(:, end)];
    end

    iterates = cell(1, maxIter);
    residual = 0;
    info = struct();
    uSeq = zeros(4, params.N);
    xPred = repmat(state, 1, params.N + 1);

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

function [ref, refHorizon] = buildReferenceHorizon(path, idx, dt, N, yawRef)
    ref.pos = zeros(3, N);
    ref.vel = zeros(3, N);
    ref.yaw = yawRef * ones(1, N);
    ref.yaw_rate = zeros(1, N);
    refHorizon = zeros(3, N);
    for j = 1:N
        idxJ = min(idx + j - 1, size(path, 1));
        posRef = path(idxJ, :)';
        velRef = pathVelocity(path, idxJ, dt);
        ref.pos(:, j) = posRef;
        ref.vel(:, j) = velRef;
        refHorizon(:, j) = posRef;
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

function log = initLog(n)
    log = struct();
    log.time = zeros(1, n);
    log.state = zeros(8, n);
    log.pos = zeros(3, n);
    log.vel = zeros(3, n);
    log.euler = nan(3, n);
    log.yawRate = zeros(1, n);
    log.ref = zeros(3, n);
    log.u = zeros(4, n);
    log.success = false(1, n);
    log.solveTime = zeros(1, n);
    log.err = zeros(1, n);
    log.pathIdx = zeros(1, n);
    log.predHorizon = cell(1, n);
    log.constraintMetadata = cell(1, n);
    log.slackMean = zeros(1, n);
    log.slackMax = zeros(1, n);
    log.iterationCount = zeros(1, n);
    log.convergenceResidual = zeros(1, n);
    log.constraintCount = zeros(1, n);
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

function value = getStructValue(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
