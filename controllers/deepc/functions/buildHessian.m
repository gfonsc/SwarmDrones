function H = buildHessian(Uf, Yf, Yp, nG, params)
% BUILDHESSIAN  Build the constant Hessian matrix for the DeePC QP.
%
%   H = buildHessian(Uf, Yf, Yp, nG, params)
%
%   The DeePC optimization (Eq. 5.5) with quadratic cost (Eq. 5.6) and
%   L2 regularization (Eq. 5.7, q=2) leads to the QP:
%
%       min  1/2 g' H g + f' g
%        g
%
%   The Hessian H collects all quadratic-in-g terms:
%
%       H = 2 * [ Yf' Q Yf          ... tracking cost  (y - y_r)' Q (y - y_r)
%               + Uf' R Uf          ... input cost     (u - u_r)' R (u - u_r)
%               + lambda_y Yp' Yp   ... initial cond.  lambda_y ||Y_p g - y_ini||^2
%               + lambda_g I ]      ... regularization lambda_g ||g - g_r||^2
%
%   The factor of 2 arises from expanding quadratic forms into the
%   standard QP format min 1/2 x'Hx + f'x.
%
%   H is CONSTANT across control steps — it does not depend on u_ini,
%   y_ini, or the reference trajectory r.
%
%   Inputs:
%       Uf     - Future input Hankel matrix  (m*T_f x nG)
%       Yf     - Future output Hankel matrix (p*T_f x nG)
%       Yp     - Past output Hankel matrix   (p*T_ini x nG)
%       nG     - Number of decision variables (= number of Hankel columns)
%       params - Struct with Q_full, R_full, lambda_y, lambda_g, epsReg
%
%   Output:
%       H - Hessian matrix (nG x nG), symmetric positive definite
%
%   Reference: Coulson thesis, Eq. (5.5), (5.6), (5.7)

    Q = params.Q_full;      % (p*T_f x p*T_f)
    R = params.R_full;      % (m*T_f x m*T_f)
    lambda_y = params.lambda_y;
    lambda_g = params.lambda_g;
    epsReg   = params.epsReg;

    %% Build Hessian — each term corresponds to a part of the objective
    %
    %  Term 1: Tracking cost   (y - y_r)' Q (y - y_r)
    %  Since y = Yf*g, this contributes: g' (Yf' Q Yf) g
    H_tracking = Yf' * Q * Yf;

    %  Term 2: Input cost   (u - u_r)' R (u - u_r)
    %  Since u = Uf*g, this contributes: g' (Uf' R Uf) g
    H_input = Uf' * R * Uf;

    %  Term 3: Initial condition penalty   lambda_y ||Yp g - y_ini||^2
    %  This contributes: lambda_y * g' (Yp' Yp) g
    H_initCond = lambda_y * (Yp' * Yp);

    %  Term 4: Regularization   lambda_g ||g - g_r||^2  (for q=2)
    %  This contributes: lambda_g * g' I g
    H_reg = lambda_g * eye(nG);

    %% Assemble full Hessian (with factor 2 for QP standard form)
    H = 2 * (H_tracking + H_input + H_initCond + H_reg);

    %% Add small numerical regularization for conditioning
    H = H + epsReg * eye(nG);

    %% Ensure perfect symmetry (guards against floating-point asymmetry)
    H = (H + H') / 2;
end
