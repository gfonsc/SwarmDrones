function [Aeq, Aineq, bIneq] = buildConstraintMatrix(Up, Uf, Yf, params)
% UILDCONSTRAINTMATRIX  Build the constant constraint matrices for DeePC QP.
%
%   [Aeq, Aineq, bIneq] = buildConstraintMatrix(Up, Uf, Yf, params)
%
%   From the DeePC optimization (Eq. 5.5):
%
%   EQUALITY CONSTRAINTS:
%       U_p g = u_ini       (hard — input initial condition)
%
%       Note: Y_p g ≈ y_ini is NOT a hard constraint in Eq. (5.5).
%       It is softened into the objective via lambda_y ||Y_p g - y_ini||^2.
%
%   INEQUALITY CONSTRAINTS (box constraints on u and y):
%       u_min <= U_f g <= u_max     (input bounds)
%       y_min <= Y_f g <= y_max     (output bounds)
%
%       Rewritten as:    Uf * g  <=  u_max
%                       -Uf * g  <= -u_min
%                        Yf * g  <=  y_max
%                       -Yf * g  <= -y_min
%
%   All constraint matrices are CONSTANT — only the equality RHS (u_ini)
%   changes per step.
%
%   Inputs:
%       Up     - Past input Hankel (m*T_ini x nG)
%       Uf     - Future input Hankel (m*T_f x nG)
%       Yf     - Future output Hankel (p*T_f x nG)
%       params - Struct with u_min_full, u_max_full, y_min_full, y_max_full
%
%   Outputs:
%       Aeq    - Equality constraint matrix (m*T_ini x nG) = Up
%       Aineq  - Inequality constraint matrix (2*(m+p)*T_f x nG)
%       bIneq  - Inequality RHS vector (2*(m+p)*T_f x 1)
%
%   Reference: Coulson thesis, Eq. (5.5) — constraint structure

    %% Equality: U_p g = u_ini (matrix is constant, RHS changes per step)
    Aeq = Up;

    %% Inequality: box constraints on u = Uf*g and y = Yf*g
    %   [  Uf ] g <= [  u_max ]
    %   [ -Uf ] g <= [ -u_min ]
    %   [  Yf ] g <= [  y_max ]
    %   [ -Yf ] g <= [ -y_min ]
    Aineq = [ Uf;
             -Uf;
              Yf;
             -Yf];

    bIneq = [ params.u_max_full;
             -params.u_min_full;
              params.y_max_full;
             -params.y_min_full];
end
