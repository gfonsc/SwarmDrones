function gRef = computeSteadyStateG(Up, Yp, Uf, Yf, params)
% COMPUTESTEADYSTATEG  Compute the steady-state trajectory mapper g_r.
%
%   gRef = computeSteadyStateG(Up, Yp, Uf, Yf, params)
%
%   Computes g_r as defined in Eq. (5.7) of the Coulson thesis:
%
%       g_r = pinv([Up; Yp; Uf; Yf]) * [1_{T_ini} ⊗ u_r; 1_{T_ini} ⊗ y_r;
%                                         1_{T_f}   ⊗ u_r; 1_{T_f}   ⊗ y_r]
%
%   The vector g_r is the minimum-norm g that maps the Hankel matrix columns
%   to the steady-state reference trajectory (u_r, y_r). Penalizing
%   ||g - g_r|| ensures that the cost is zero at steady state.
%
%   Inputs:
%       Up, Yp, Uf, Yf - Hankel matrices from buildHankelMatrix
%       params          - Struct containing u_r, y_r, T_ini, T_f
%
%   Output:
%       gRef - Steady-state g vector (numCols x 1)
%
%   Reference: Coulson thesis, Eq. (5.7) — Regularization function

    T_ini = params.T_ini;
    T_f   = params.T_f;
    u_r   = params.u_r;   % (m x 1)
    y_r   = params.y_r;   % (p x 1)

    %% Build the steady-state trajectory vector
    %  [1_{T_ini} ⊗ u_r]   means u_r repeated T_ini times
    %  [1_{T_ini} ⊗ y_r]   means y_r repeated T_ini times
    %  [1_{T_f}   ⊗ u_r]   means u_r repeated T_f times
    %  [1_{T_f}   ⊗ y_r]   means y_r repeated T_f times
    steadyStateTrajectory = [repmat(u_r, T_ini, 1);   % m*T_ini x 1
                             repmat(y_r, T_ini, 1);   % p*T_ini x 1
                             repmat(u_r, T_f,   1);   % m*T_f   x 1
                             repmat(y_r, T_f,   1)];  % p*T_f   x 1

    %% Build the full stacked Hankel matrix
    fullHankel = [Up; Yp; Uf; Yf];

    %% Compute g_r as the minimum-norm solution — Eq. (5.7)
    %  g_r = pinv(fullHankel) * steadyStateTrajectory
    %  This is the g with smallest ||g||_2 that satisfies (or best approximates):
    %  fullHankel * g = steadyStateTrajectory
    gRef = pinv(fullHankel) * steadyStateTrajectory;

    if params.verbose
        residual = norm(fullHankel * gRef - steadyStateTrajectory);
        fprintf('  Steady-state g_r: ||g_r|| = %.4f, residual = %.2e\n', ...
            norm(gRef), residual);
    end
end
