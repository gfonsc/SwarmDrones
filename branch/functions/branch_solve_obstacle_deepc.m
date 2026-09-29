function [uOpt, yPred, info] = branch_solve_obstacle_deepc(config, uIni, yIni, r, pNom, obstacleOptions)
%BRANCH_SOLVE_OBSTACLE_DEEPC Solve one DeePC QP augmented with soft obstacle constraints.

    rhoObs = getOption(obstacleOptions, 'RhoObs', 1e4);

    [A_obs, b_obs, metadata] = branch_build_obstacle_constraints(config, pNom, obstacleOptions);
    if isempty(A_obs)
        [uOpt, yPred, baseInfo] = deepcAlgorithm(config.model, uIni, yIni, r, config.solverOpts);
        baseInfo.slack = [];
        baseInfo.constraintMetadata = metadata;
        baseInfo.constraintCount = 0;
        baseInfo.iterationsUsed = 1;
        baseInfo.convergenceResidual = 0;
        baseInfo.predictedPositions = predictionPositions(config, yPred);
        info = baseInfo;
        return;
    end

    H_g = config.model.H;
    f_g = buildLinearCost(config.model.precomp, r, yIni);
    nG = config.model.dims.nG;
    nSlack = size(A_obs, 1);

    H_aug = blkdiag(H_g, 2 * rhoObs * eye(nSlack));
    H_aug = (H_aug + H_aug') / 2;
    f_aug = [f_g; zeros(nSlack, 1)];

    Aeq_aug = [config.model.Aeq, zeros(size(config.model.Aeq, 1), nSlack)];
    beq = uIni;

    Aineq_aug = [
        config.model.Aineq, zeros(size(config.model.Aineq, 1), nSlack);
        A_obs, -eye(nSlack);
        zeros(nSlack, nG), -eye(nSlack)
    ];
    bineq_aug = [
        config.model.bIneq;
        b_obs;
        zeros(nSlack, 1)
    ];

    [zOpt, fval, exitflag, solverInfo] = solveQP( ...
        H_aug, f_aug, Aeq_aug, beq, Aineq_aug, bineq_aug, config.solverOpts);

    info = struct();
    info.exitflag = exitflag;
    info.fval = fval;
    info.solver = solverInfo.solver;
    info.solveTime = solverInfo.solveTime;
    info.constraintMetadata = metadata;
    info.constraintCount = nSlack;
    info.iterationsUsed = 1;
    info.convergenceResidual = 0;
    info.success = (exitflag == 1);

    if isfield(solverInfo, 'solsta')
        info.solsta = solverInfo.solsta;
    end
    if isfield(solverInfo, 'prosta')
        info.prosta = solverInfo.prosta;
    end
    if isfield(solverInfo, 'iterations')
        info.iterations = solverInfo.iterations;
    end
    if isfield(solverInfo, 'algorithm')
        info.algorithm = solverInfo.algorithm;
    end

    if info.success
        gOpt = zOpt(1:nG);
        slack = zOpt(nG + 1:end);
        uOpt = config.model.Uf * gOpt;
        yPred = config.model.Yf * gOpt;
        info.g = gOpt;
        info.slack = slack;
    else
        uOpt = zeros(config.m * config.T_f, 1);
        yPred = zeros(config.p * config.T_f, 1);
        info.g = zeros(nG, 1);
        info.slack = zeros(nSlack, 1);
    end

    info.predictedPositions = predictionPositions(config, yPred);
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

function value = getOption(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
