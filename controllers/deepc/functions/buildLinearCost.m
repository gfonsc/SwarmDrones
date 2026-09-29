function f = buildLinearCost(precomp, r, yIni)
% BUILDLINEARCOST  Build the linear cost vector f for the DeePC QP.
%
%   f = buildLinearCost(precomp, r, yIni)
%
%   This function computes the ONLY part of the QP that changes each step.
%   The linear cost collects all linear-in-g terms from the expanded objective:
%
%       f = -2 * [ Yf' Q r              ... varies with reference trajectory
%                + Uf' R u_r_full       ... constant (precomputed)
%                + lambda_y Yp' y_ini   ... varies with current measurements
%                + lambda_g g_r ]       ... constant (precomputed)
%
%   Where:
%       r      = y_r_full for regulation, or time-varying reference (p*T_f x 1)
%       y_ini  = most recent p*T_ini output measurements
%
%   Inputs:
%       precomp - Struct from deepcSetup containing precomputed products:
%                   .neg2_YfQ       = -2 * Yf' * Q  (nG x p*T_f)
%                   .neg2_lambdaY_YpT = -2 * lambda_y * Yp'  (nG x p*T_ini)
%                   .fConst         = -2*(Uf'*R*u_r_full + lambda_g*g_r) (nG x 1)
%       r       - Reference output trajectory for this step (p*T_f x 1)
%       yIni    - Recent output measurements (p*T_ini x 1)
%
%   Output:
%       f - Linear cost vector (nG x 1)
%
%   Reference: Coulson thesis, Eq. (5.5), (5.6), (5.7)

    %% Compute varying terms + add precomputed constant terms
    %
    %  f = (-2 Yf'Q) * r + (-2 lambda_y Yp') * y_ini + fConst
    %
    %  This is an efficient matrix-vector multiply: O(nG * p*T_f) per step.
    f = precomp.neg2_YfQ * r ...         % Tracking: -2 Yf' Q r
      + precomp.neg2_lambdaY_YpT * yIni ... % Initial cond: -2 lambda_y Yp' y_ini
      + precomp.fConst;                  % Constants: -2 (Uf'R u_r + lambda_g g_r)
end
