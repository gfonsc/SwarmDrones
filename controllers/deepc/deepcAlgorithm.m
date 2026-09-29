function [uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r, solverOpts)
% DEEPCALGORITHM  Online step of DeePC — solve QP and return optimal inputs.
%
%   [uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r)
%   [uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r, solverOpts)
%
%   Implements Algorithm 5.1 (Steps 1-3) from Coulson [2021]:
%
%     Step 1: Set u_ini and y_ini from recent I/O measurements
%     Step 2: Solve the regularized DeePC optimization (Eq. 5.5)
%     Step 3: Return u* = U_f g* and y* = Y_f g*
%
%   The caller is responsible for:
%     - Applying u*(1:m) to the plant (receding horizon, Step 2 of Alg. 5.1)
%     - Updating uIni and yIni with new measurements (Step 3 of Alg. 5.1)
%     - Providing the reference trajectory r for the current prediction window
%
%   Optimization problem solved (Eq. 5.5):
%
%     min  (Yf*g - r)' Q (Yf*g - r)           ... output tracking
%      g
%        + (Uf*g - u_r)' R (Uf*g - u_r)       ... input penalty
%        + lambda_y || Yp*g - y_ini ||^2       ... initial condition (softened)
%        + lambda_g || g - g_r ||^2            ... regularization (Eq. 5.7)
%
%     s.t. Up * g = u_ini                      ... hard input initial condition
%          u_min <= Uf * g <= u_max            ... input constraints
%          y_min <= Yf * g <= y_max            ... output constraints
%
%   This function only computes what CHANGES per step:
%     - Linear cost vector f (depends on r and yIni)
%     - Equality RHS beq (depends on uIni)
%   Everything else was precomputed in deepcSetup().
%
%   Inputs:
%       model      - Struct from deepcSetup() containing precomputed data
%       uIni       - Recent input measurements (m*T_ini x 1)
%       yIni       - Recent output measurements (p*T_ini x 1)
%       r          - Reference output trajectory (p*T_f x 1)
%                    For regulation: repmat(y_r, T_f, 1)
%                    For tracking:   stacked reference over prediction window
%       solverOpts - (Optional) Struct with:
%                      .solver  - 'mosek', 'quadprog', or 'auto'
%                      .verbose - Solver verbosity
%
%   Outputs:
%       uOpt  - Optimal input trajectory (m*T_f x 1)
%               Apply uOpt(1:m) to the plant (receding horizon)
%       yPred - Predicted output trajectory (p*T_f x 1)
%       info  - Struct with solver details:
%                .success, .fval, .g, .solver, .solveTime, .exitflag
%
%   Example (inside control loop):
%       [uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r);
%       uApply = uOpt(1:m);              % First input only
%       yMeas  = plantStep(x, uApply);   % Apply to plant
%       uIni   = [uIni(m+1:end); uApply]; % Shift input buffer
%       yIni   = [yIni(p+1:end); yMeas];  % Shift output buffer
%
%   Reference: Coulson thesis, Algorithm 5.1

    %% Default solver options
    if nargin < 5
        solverOpts = struct('solver', 'auto', 'verbose', false);
    end

    %% Unpack model (precomputed in deepcSetup)
    H       = model.H;
    Aeq     = model.Aeq;
    Aineq   = model.Aineq;
    bIneq   = model.bIneq;
    Uf      = model.Uf;
    Yf      = model.Yf;
    precomp = model.precomp;
    params  = model.params;

    %% Step 1: Build the linear cost vector f (ONLY changing part)
    %  f = -2*(Yf'*Q*r + lambda_y*Yp'*yIni + Uf'*R*u_r + lambda_g*g_r)
    %  Using precomputed products for speed:
    f = buildLinearCost(precomp, r, yIni);

    %% Step 2: Set equality constraint RHS
    %  U_p g = u_ini (hard constraint on past inputs)
    beq = uIni;

    %% Step 3: Solve the QP — Eq. (5.5)
    [zOpt, fval, exitflag, solverInfo] = solveQP(H, f, Aeq, beq, ...
        Aineq, bIneq, solverOpts);

    %% Step 4: Extract u* = Uf*g* and y* = Yf*g*
    [uOpt, yPred, info] = extractSolution(zOpt, Uf, Yf, fval, exitflag, ...
        solverInfo, params);
end