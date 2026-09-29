function [uSeq, xPred, info] = mpc_solve_tracking(model, x0, ref, params, varargin)
%MPC_SOLVE_TRACKING Solve the branch-side condensed MPC tracking QP.

    p = inputParser;
    addParameter(p, 'ExtraConstraints', struct(), @isstruct);
    parse(p, varargin{:});
    extra = p.Results.ExtraConstraints;

    params = fillDefaults(params, model.defaultParams);
    [Sx, Su] = buildPredictionMatrices(model.A, model.B, params.N);
    xRef = buildStateReference(ref, params.N, x0);

    [H_u, f_u] = buildCondensedCost(model, Sx, Su, x0, xRef, params);
    nU = size(H_u, 1);

    Aineq = [];
    bineq = [];
    if isfield(extra, 'A') && ~isempty(extra.A)
        extraA = extra.A;
        extraB = extra.b;
    else
        extraA = zeros(0, nU);
        extraB = zeros(0, 1);
    end

    boundA = [eye(nU); -eye(nU)];
    boundB = [repmat(params.u_max(:), params.N, 1); -repmat(params.u_min(:), params.N, 1)];

    if isfield(extra, 'useSlack') && extra.useSlack
        nSlack = size(extraA, 1);
        rhoSlack = getStructValue(extra, 'rhoSlack', 1e4);
        H = blkdiag(H_u, 2 * rhoSlack * eye(nSlack));
        H = (H + H') / 2;
        f = [f_u; zeros(nSlack, 1)];
        Aineq = [
            boundA, zeros(size(boundA, 1), nSlack);
            extraA, -eye(nSlack);
            zeros(nSlack, nU), -eye(nSlack)
        ];
        bineq = [
            boundB;
            extraB;
            zeros(nSlack, 1)
        ];
    else
        H = H_u;
        H = (H + H') / 2;
        f = f_u;
        Aineq = [
            boundA;
            extraA
        ];
        bineq = [
            boundB;
            extraB
        ];
        nSlack = 0;
    end

    Aeq = sparse(0, size(H, 1));
    beq = zeros(0, 1);

    [zOpt, fval, exitflag, solverInfo] = solveQP(H, f, Aeq, beq, Aineq, bineq, params.solverOpts);

    info = struct();
    info.exitflag = exitflag;
    info.fval = fval;
    info.solver = solverInfo.solver;
    info.solveTime = solverInfo.solveTime;
    info.success = (exitflag == 1);
    info.Sx = Sx;
    info.Su = Su;
    info.referenceState = xRef;
    info.constraintCount = size(extraA, 1);
    info.constraintMetadata = getStructValue(extra, 'metadata', struct());

    if isfield(solverInfo, 'iterations')
        info.iterations = solverInfo.iterations;
    end
    if isfield(solverInfo, 'algorithm')
        info.algorithm = solverInfo.algorithm;
    end

    if info.success
        U = zOpt(1:nU);
    else
        U = zeros(nU, 1);
    end

    if nSlack > 0
        info.slack = zOpt(nU + 1:end);
    else
        info.slack = [];
    end

    uSeq = reshape(U, [], params.N);
    X = Sx * x0 + Su * U;
    xPred = [x0, reshape(X, size(model.A, 1), params.N)];
    info.predictedPositions = xPred(1:3, 2:end);
end

function params = fillDefaults(params, defaults)
    fields = fieldnames(defaults);
    for i = 1:numel(fields)
        if ~isfield(params, fields{i}) || isempty(params.(fields{i}))
            params.(fields{i}) = defaults.(fields{i});
        end
    end
end

function [Sx, Su] = buildPredictionMatrices(A, B, N)
    n = size(A, 1);
    m = size(B, 2);

    Sx = zeros(n * N, n);
    Su = zeros(n * N, m * N);
    A_pow = eye(n);
    for k = 1:N
        A_pow = A * A_pow;
        rows = (k - 1) * n + (1:n);
        Sx(rows, :) = A_pow;
        for j = 1:k
            cols = (j - 1) * m + (1:m);
            Su(rows, cols) = A^(k - j) * B;
        end
    end
end

function xRef = buildStateReference(ref, N, x0)
    posRef = padHorizon(ref.pos, N);
    velRef = padHorizon(ref.vel, N);
    if isfield(ref, 'yaw')
        yawRef = padScalarHorizon(ref.yaw, N);
    else
        yawRef = repmat(x0(7), 1, N);
    end
    if isfield(ref, 'yaw_rate')
        yawRateRef = padScalarHorizon(ref.yaw_rate, N);
    else
        yawRateRef = zeros(1, N);
    end

    xRef = zeros(8, N);
    xRef(1:3, :) = posRef;
    xRef(4:6, :) = velRef;
    xRef(7, :) = yawRef;
    xRef(8, :) = yawRateRef;
end

function Y = padHorizon(Y, N)
    if isempty(Y)
        Y = zeros(3, N);
        return;
    end
    if size(Y, 1) ~= 3
        Y = Y';
    end
    if size(Y, 2) < N
        Y = [Y, repmat(Y(:, end), 1, N - size(Y, 2))];
    else
        Y = Y(:, 1:N);
    end
end

function y = padScalarHorizon(y, N)
    y = y(:)';
    if isempty(y)
        y = zeros(1, N);
    elseif numel(y) < N
        y = [y, repmat(y(end), 1, N - numel(y))];
    else
        y = y(1:N);
    end
end

function [H, f] = buildCondensedCost(model, Sx, Su, x0, xRef, params)
    n = size(model.A, 1);
    N = params.N;

    Q = blkdiag(params.Q_pos, params.Q_vel, params.Q_yaw, params.Q_yaw_rate);
    Qbar = kron(eye(N), Q);
    Qbar(end - n + 1:end, end - n + 1:end) = params.P_term * Q;

    Rbar = kron(eye(N), params.R);

    m = size(model.B, 2);
    D = zeros(m * N, m * N);
    for k = 1:N
        rows = (k - 1) * m + (1:m);
        cols = (k - 1) * m + (1:m);
        D(rows, cols) = eye(m);
        if k > 1
            prevCols = (k - 2) * m + (1:m);
            D(rows, prevCols) = -eye(m);
        end
    end
    d = zeros(m * N, 1);
    d(1:m) = -params.u_prev(:);
    RdeltaBar = kron(eye(N), params.R_delta);

    xRefVec = xRef(:);
    H = Su' * Qbar * Su + Rbar + D' * RdeltaBar * D + 1e-8 * eye(m * N);
    H = (H + H') / 2;
    f = 2 * Su' * Qbar * (Sx * x0 - xRefVec) + 2 * D' * RdeltaBar * d;
end

function value = getStructValue(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
