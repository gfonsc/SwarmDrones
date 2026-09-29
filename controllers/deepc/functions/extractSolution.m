function [uOpt, yPred, info] = extractSolution(zOpt, Uf, Yf, fval, exitflag, solverInfo, params)
% EXTRACTSOLUTION  Extract optimal u and y from the QP solution g*.
%
%   [uOpt, yPred, info] = extractSolution(zOpt, Uf, Yf, fval, exitflag, solverInfo, params)
%
%   Given the optimal decision variable g* from the QP, recovers:
%       u* = U_f g*     (optimal future input trajectory, m*T_f x 1)
%       y* = Y_f g*     (predicted future output trajectory, p*T_f x 1)
%
%   For receding horizon control (Algorithm 5.1, Step 2), only the first
%   input u*(1:m) is applied to the plant.
%
%   Inputs:
%       zOpt       - Optimal g vector (nG x 1)
%       Uf         - Future input Hankel (m*T_f x nG)
%       Yf         - Future output Hankel (p*T_f x nG)
%       fval       - Optimal objective value
%       exitflag   - Solver exit flag (1 = success)
%       solverInfo - Struct from solveQP with solver details
%       params     - Struct with m, p, T_f
%
%   Outputs:
%       uOpt  - Optimal input trajectory (m*T_f x 1)
%       yPred - Predicted output trajectory (p*T_f x 1)
%       info  - Struct with:
%                .success   - Boolean: was the QP solved successfully?
%                .fval      - Optimal objective value
%                .g         - Optimal g vector
%                .solver    - Name of solver used
%                .solveTime - Time to solve (seconds)
%                .exitflag  - Raw solver exit flag
%
%   Reference: Coulson thesis, Algorithm 5.1, Steps 1-2

    info = struct();
    info.exitflag  = exitflag;
    info.fval      = fval;
    info.g         = zOpt;
    info.solver    = solverInfo.solver;
    info.solveTime = solverInfo.solveTime;
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

    %% Check success
    info.success = (exitflag == 1);

    if info.success
        %% Recover u and y from g — this is the key DeePC equation
        %  u = U_f g*     (predicted optimal inputs)
        %  y = Y_f g*     (predicted optimal outputs)
        uOpt  = Uf * zOpt;
        yPred = Yf * zOpt;
    else
        %% Solver failed — return zeros with warning
        m   = params.m;
        p   = params.p;
        T_f = params.T_f;

        uOpt  = zeros(m * T_f, 1);
        yPred = zeros(p * T_f, 1);

        warning('deepc:solverFailed', ...
            'QP solver "%s" failed (exitflag=%d). Returning zero inputs.', ...
            solverInfo.solver, exitflag);
    end
end
