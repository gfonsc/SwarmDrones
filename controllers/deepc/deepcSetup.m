function model = deepcSetup(uData, yData, params)
% DEEPCSETUP  Offline phase of DeePC — build Hankel matrices and precompute QP.
%
%   model = deepcSetup(uData, yData, params)
%
%   Performs all OFFLINE computations for the Data-Enabled Predictive
%   Controller (DeePC) as described in Coulson [2021], Algorithm 5.1.
%
%   This function is called ONCE with the training data. It:
%     1. Validates parameters and sets defaults (Section 5.3.2 guidelines)
%     2. Builds Hankel matrices and partitions into past/future (Eq. 5.4)
%     3. Checks persistency of excitation (Eq. 5.9)
%     4. Computes the steady-state reference g_r (Eq. 5.7)
%     5. Precomputes the constant Hessian H (Eq. 5.5 + 5.6)
%     6. Precomputes constraint matrices
%     7. Precomputes partial products for fast linear cost updates
%
%   After setup, call deepcAlgorithm(model, uIni, yIni, r) every step.
%
%   Inputs:
%       uData  - Training input data (m x T)
%                m = number of inputs, T = number of time samples
%       yData  - Training output data (p x T)
%                p = number of outputs
%       params - Hyperparameter struct (see validateParams.m for full list)
%                Required: T_ini, T_f
%                Recommended: Q, R, lambda_y, lambda_g, u_r, y_r
%
%   Output:
%       model - Struct containing all precomputed data for online use:
%                .Up, .Yp, .Uf, .Yf   - Hankel matrices
%                .H                   - Constant Hessian (nG x nG)
%                .Aeq                 - Equality constraint matrix
%                .Aineq, .bIneq       - Inequality constraint matrix/RHS
%                .gRef                - Steady-state g_r
%                .precomp             - Precomputed products for buildLinearCost
%                .params              - Validated parameter struct
%                .dims                - Dimension struct (m, p, T_ini, T_f, nG)
%
%   Example:
%       params.T_ini = 6;  params.T_f = 25;
%       params.lambda_y = 7.5e8;  params.lambda_g = 500;
%       model = deepcSetup(u_train, y_train, params);
%
%       for t = 1:T_sim
%           [uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r);
%           uApply = uOpt(1:m);  % Receding horizon
%           ...
%       end
%
%   Reference: Coulson thesis, Algorithm 5.1 and Section 5.2.3

    %% Add functions folder to path
    funcPath = fullfile(fileparts(mfilename('fullpath')), 'functions');
    addpath(funcPath);

    %% Extract dimensions
    [m, T] = size(uData);
    [p, ~] = size(yData);

    fprintf('====================================================\n');
    fprintf('  DeePC Offline Setup\n');
    fprintf('====================================================\n');
    fprintf('  Inputs (m):  %d\n', m);
    fprintf('  Outputs (p): %d\n', p);
    fprintf('  Data length (T): %d samples\n', T);

    %% Step 1: Validate and complete parameters
    fprintf('\n--- Step 1: Validating parameters ---\n');
    params = validateParams(params, m, p);

    T_ini = params.T_ini;
    T_f   = params.T_f;

    fprintf('  T_ini = %d (initial trajectory length)\n', T_ini);
    fprintf('  T_f   = %d (prediction horizon)\n', T_f);
    fprintf('  lambda_y = %.2e (initial condition penalty)\n', params.lambda_y);
    fprintf('  lambda_g = %.1f (g regularization, qNorm=%d)\n', params.lambda_g, params.qNorm);

    %% Step 2: Build Hankel matrices — Eq. (5.4)
    fprintf('\n--- Step 2: Building Hankel matrices (Eq. 5.4) ---\n');
    [Up, Yp, Uf, Yf] = buildHankelMatrix(uData, yData, T_ini, T_f);

    nG = size(Up, 2);  % Number of columns = dimension of g

    fprintf('  Up: %d x %d (past inputs)\n', size(Up, 1), size(Up, 2));
    fprintf('  Yp: %d x %d (past outputs)\n', size(Yp, 1), size(Yp, 2));
    fprintf('  Uf: %d x %d (future inputs)\n', size(Uf, 1), size(Uf, 2));
    fprintf('  Yf: %d x %d (future outputs)\n', size(Yf, 1), size(Yf, 2));
    fprintf('  Decision variable g: %d x 1\n', nG);

    %% Step 3: Check persistency of excitation — Eq. (5.9)
    fprintf('\n--- Step 3: Persistency of excitation check (Eq. 5.9) ---\n');
    checkPersistency(Up, Yp, Uf, uData, params);

    %% Step 4: Compute steady-state g_r — Eq. (5.7)
    fprintf('\n--- Step 4: Computing g_r (Eq. 5.7) ---\n');
    gRef = computeSteadyStateG(Up, Yp, Uf, Yf, params);

    %% Step 5: Precompute Hessian — Eq. (5.5) + (5.6)
    fprintf('\n--- Step 5: Building Hessian (Eq. 5.5-5.6) ---\n');
    H = buildHessian(Uf, Yf, Yp, nG, params);

    condH = cond(H);
    fprintf('  Hessian size: %d x %d\n', size(H, 1), size(H, 2));
    fprintf('  Condition number: %.2e\n', condH);
    if condH > 1e12
        warning('deepc:illConditioned', ...
            'Hessian condition number %.2e is very large. Consider increasing epsReg or lambda_g.', condH);
    end

    %% Step 6: Build constraint matrices
    fprintf('\n--- Step 6: Building constraints (Eq. 5.5) ---\n');
    [Aeq, Aineq, bIneq] = buildConstraintMatrix(Up, Uf, Yf, params);

    fprintf('  Equality constraints:   %d (U_p g = u_ini)\n', size(Aeq, 1));
    fprintf('  Inequality constraints: %d (u/y bounds)\n', size(Aineq, 1));

    %% Step 7: Precompute partial products for fast linear cost updates
    fprintf('\n--- Step 7: Precomputing products for online phase ---\n');

    precomp = struct();

    %  -2 * Yf' * Q  (used: neg2_YfQ * r, changes each step)
    precomp.neg2_YfQ = -2 * Yf' * params.Q_full;

    %  -2 * lambda_y * Yp' (used: neg2_lambdaY_YpT * yIni, changes each step)
    precomp.neg2_lambdaY_YpT = -2 * params.lambda_y * Yp';

    %  Constant part of f: -2 * (Uf'*R*u_r_full + lambda_g*g_r)
    precomp.fConst = -2 * (Uf' * params.R_full * params.u_r_full ...
                         + params.lambda_g * gRef);

    fprintf('  Precomputed: neg2_YfQ (%d x %d)\n', size(precomp.neg2_YfQ));
    fprintf('  Precomputed: neg2_lambdaY_YpT (%d x %d)\n', size(precomp.neg2_lambdaY_YpT));
    fprintf('  Precomputed: fConst (%d x 1)\n', length(precomp.fConst));

    %% Pack everything into model struct
    model = struct();

    % Hankel matrices
    model.Up = Up;
    model.Yp = Yp;
    model.Uf = Uf;
    model.Yf = Yf;

    % QP components (constant)
    model.H      = H;
    model.Aeq    = Aeq;
    model.Aineq  = Aineq;
    model.bIneq  = bIneq;

    % Precomputed products (for fast online linear cost)
    model.precomp = precomp;

    % Reference
    model.gRef = gRef;

    % Parameters
    model.params = params;

    % Dimensions (for easy access)
    model.dims = struct('m', m, 'p', p, 'T_ini', T_ini, 'T_f', T_f, 'nG', nG, 'T', T);

    fprintf('\n====================================================\n');
    fprintf('  DeePC Setup Complete\n');
    fprintf('  Ready for online control with deepcAlgorithm()\n');
    fprintf('====================================================\n');
end