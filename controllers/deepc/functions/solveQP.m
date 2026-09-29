function [zOpt, fval, exitflag, solverInfo] = solveQP(H, f, Aeq, beq, Aineq, bIneq, solverOpts)
% SOLVEQP  Solve the DeePC quadratic program using MOSEK or quadprog.
%
%   [zOpt, fval, exitflag, solverInfo] = solveQP(H, f, Aeq, beq, Aineq, bIneq, solverOpts)
%
%   Solves the standard QP:
%       min   1/2 x' H x + f' x
%        x
%       s.t.  Aeq * x  = beq
%             Aineq * x <= bIneq
%
%   Strategy: tries MOSEK first (faster), falls back to quadprog if
%   MOSEK is not installed.
%
%   Inputs:
%       H        - Hessian matrix (nG x nG), symmetric positive definite
%       f        - Linear cost vector (nG x 1)
%       Aeq      - Equality constraint matrix (nEq x nG)
%       beq      - Equality RHS (nEq x 1)
%       Aineq    - Inequality constraint matrix (nIneq x nG)
%       bIneq    - Inequality RHS (nIneq x 1)
%       solverOpts - Struct with optional fields:
%                     .solver  - 'mosek', 'quadprog', or 'auto' (default: 'auto')
%                     .verbose - Solver output flag (default: false)
%
%   Outputs:
%       zOpt       - Optimal decision variable (nG x 1)
%       fval       - Optimal objective value
%       exitflag   - 1 = success, 0 = iteration limit, -1 = infeasible
%       solverInfo - Struct with solver name, solve time, details

    if nargin < 7, solverOpts = struct(); end
    if ~isfield(solverOpts, 'solver'),  solverOpts.solver = 'auto';  end
    if ~isfield(solverOpts, 'verbose'), solverOpts.verbose = false;   end

    nG = size(H, 1);
    tStart = tic;

    %% Select solver
    useMosek = false;
    if strcmpi(solverOpts.solver, 'mosek')
        useMosek = true;
    elseif strcmpi(solverOpts.solver, 'auto')
        useMosek = isMosekAvailable();
    end

    %% Solve
    if useMosek
        [zOpt, fval, exitflag, solverInfo] = solveMosek(H, f, Aeq, beq, ...
            Aineq, bIneq, nG, solverOpts);
        solverInfo.solver = 'mosek';
    else
        [zOpt, fval, exitflag, solverInfo] = solveQuadprog(H, f, Aeq, beq, ...
            Aineq, bIneq, nG, solverOpts);
        solverInfo.solver = 'quadprog';
    end

    solverInfo.solveTime = toc(tStart);
end


%% ==================== MOSEK ====================
function [zOpt, fval, exitflag, info] = solveMosek(H, f, Aeq, beq, Aineq, bIneq, nG, opts)
%SOLVEMOSEK  Solve QP using MOSEK.
%
%   MOSEK QP format:  min  1/2 x' Q_o x + c' x
%                     s.t. blc <= A x <= buc
%                          blx <= x   <= bux

    info = struct();

    % Build MOSEK problem structure
    prob = struct();

    %% Objective: Hessian (lower triangle, triplet format)
    %  MOSEK expects Q_o such that objective = 1/2 x' Q_o x + c' x
    %  Our H is already in the correct form for this convention.
    [qi, qj, qv] = find(tril(H));
    prob.qosubi = qi;
    prob.qosubj = qj;
    prob.qoval  = qv;

    %% Objective: linear term
    prob.c = f;

    %% Constraints: stack equality and inequality
    %  Equality:   Aeq * x  = beq        →  blc = beq, buc = beq
    %  Inequality: Aineq * x <= bIneq    →  blc = -inf, buc = bIneq
    nEq   = size(Aeq, 1);
    nIneq = size(Aineq, 1);

    prob.a   = sparse([Aeq; Aineq]);
    prob.blc = [beq; -inf(nIneq, 1)];
    prob.buc = [beq; bIneq];

    %% Variable bounds (no bounds on g — unconstrained)
    prob.blx = -inf(nG, 1);
    prob.bux =  inf(nG, 1);

    %% Solver parameters
    param = struct();
    param.MSK_IPAR_INTPNT_MAX_ITERATIONS = 500;
    if ~opts.verbose
        echoFlag = 'minimize echo(0)';
    else
        echoFlag = 'minimize';
    end

    %% Solve
    [~, res] = mosekopt(echoFlag, prob, param);

    %% Parse result
    if isfield(res, 'sol') && isfield(res.sol, 'itr')
        sol = res.sol.itr;
        zOpt = sol.xx;
        fval = sol.pobjval;

        % Check solution status
        solstaUpper = upper(sol.solsta);
        if strcmp(solstaUpper, 'OPTIMAL') || ...
           strcmp(solstaUpper, 'NEAR_OPTIMAL')
            exitflag = 1;
        elseif (contains(solstaUpper, 'INFEAS') && contains(solstaUpper, 'CER'))
            exitflag = -1;
        else
            exitflag = 0;
        end
        info.solsta = sol.solsta;
        info.prosta = sol.prosta;
    else
        zOpt = zeros(nG, 1);
        fval = Inf;
        exitflag = -1;
        info.solsta = 'NO_SOLUTION';
        info.prosta = 'UNKNOWN';
    end
end


%% ==================== quadprog ====================
function [zOpt, fval, exitflag, info] = solveQuadprog(H, f, Aeq, beq, Aineq, bIneq, nG, opts)
%SOLVEQUADPROG  Solve QP using MATLAB's quadprog (Optimization Toolbox).
%
%   quadprog format: min 1/2 x'Hx + f'x
%                    s.t. Aineq*x <= bIneq, Aeq*x = beq

    info = struct();

    %% Set options
    if opts.verbose
        dispOpt = 'iter';
    else
        dispOpt = 'off';
    end
    options = optimoptions('quadprog', ...
        'Display', dispOpt, ...
        'MaxIterations', 500, ...
        'OptimalityTolerance', 1e-8, ...
        'ConstraintTolerance', 1e-8);

    %% Solve
    lb = [];  % No variable bounds (already in Aineq)
    ub = [];
    x0 = [];  % No warm start

    [zOpt, fval, exitflag, output] = quadprog(H, f, Aineq, bIneq, ...
        Aeq, beq, lb, ub, x0, options);

    %% Parse result
    if isempty(zOpt)
        zOpt = zeros(nG, 1);
        fval = Inf;
    end

    info.iterations = output.iterations;
    info.algorithm  = output.algorithm;

    % quadprog exitflag: 1=converged, 0=maxiter, -2=infeasible, -3=unbounded
    if exitflag ~= 1
        exitflag = min(exitflag, 0);  % Normalize: positive = success
    end
end


%% ==================== Utilities ====================
function available = isMosekAvailable()
%ISMOSEKAVAILABLE  Check if MOSEK is installed and callable.
    persistent mosekAvail
    if isempty(mosekAvail)
        try
            [~, ~] = mosekopt('symbcon echo(0)');
            mosekAvail = true;
        catch
            mosekAvail = false;
        end
    end
    available = mosekAvail;
end
