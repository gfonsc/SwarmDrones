function [u_opt, x_pred, info] = mpc_controller_mosek(A, B, C, x0, ref, Q, R, P, u_min, u_max)
%MPC_CONTROLLER_MOSEK Model Predictive Controller using MOSEK solver
%
%   Solves the MPC optimization problem:
%
%   minimize    sum_{k=0}^{N-1} ||y_k - r_k||²_Q + ||u_k||²_R
%     u
%
%   subject to  x_{k+1} = A*x_k + B*u_k
%               y_k = C*x_k
%               u_min <= u_k <= u_max
%
%   Inputs:
%       A, B, C - Discrete state-space matrices
%       x0      - Current state (n x 1)
%       ref     - Reference trajectory (p x N)
%       Q       - Output tracking weight (p x p)
%       R       - Input penalty weight (m x m)
%       P       - Prediction horizon
%       u_min, u_max - Input bounds (m x 1)
%
%   Outputs:
%       u_opt  - Optimal input sequence (m x P)
%       x_pred - Predicted state trajectory (n x P)
%       info   - Solver information

%% Dimensions
n = size(A, 1);  % States
m = size(B, 2);  % Inputs
p = size(C, 1);  % Outputs
N = P;           % Horizon

%% Extend weights and reference
if size(ref, 2) < N
    ref = [ref, repmat(ref(:,end), 1, N - size(ref, 2))];
end

%% Build prediction matrices
% x_k = A^k * x0 + sum_{j=0}^{k-1} A^{k-1-j} * B * u_j
% Formulate as: X = Sx * x0 + Su * U
% where X = [x_1; x_2; ...; x_N], U = [u_0; u_1; ...; u_{N-1}]

% State prediction matrix
Sx = zeros(n * N, n);
Su = zeros(n * N, m * N);

A_pow = eye(n);
for k = 1:N
    A_pow = A * A_pow;
    Sx((k-1)*n+1:k*n, :) = A_pow;

    for j = 1:k
        Su((k-1)*n+1:k*n, (j-1)*m+1:j*m) = A^(k-j) * B;
    end
end

% Output prediction: Y = C_bar * X = C_bar * (Sx*x0 + Su*U)
C_bar = kron(eye(N), C);

Cy_x0 = C_bar * Sx * x0;  % Free response
Cy_u = C_bar * Su;        % Forced response

%% Build QP matrices
% Cost: J = ||Y - R||²_Q + ||U||²_R
%     = U' * (Cy_u' * Q_bar * Cy_u + R_bar) * U + 2 * (Cy_x0 - R)' * Q_bar * Cy_u * U + const

Q_bar = kron(eye(N), Q);
R_bar = kron(eye(N), R);

r_vec = reshape(ref, [], 1);  % Stack reference

% Hessian
H = Cy_u' * Q_bar * Cy_u + R_bar;
H = (H + H') / 2;  % Ensure symmetry

% Add regularization for numerical stability
H = H + 1e-6 * eye(size(H));

% Linear term
f = 2 * Cy_u' * Q_bar * (Cy_x0 - r_vec);

%% Constraints
% u_min <= u_k <= u_max for all k
lb = repmat(u_min, N, 1);
ub = repmat(u_max, N, 1);

%% Solve with MOSEK
prob = struct();

% Quadratic term
[prob.qosubi, prob.qosubj, prob.qoval] = find(tril(H));

% Linear term
prob.c = f;

% Variable bounds
prob.blx = lb;
prob.bux = ub;

% No linear constraints
prob.a = sparse(0, m * N);
prob.blc = [];
prob.buc = [];

% Solve
param = struct();
param.MSK_IPAR_LOG = 0;

[~, res] = mosekopt('minimize echo(0)', prob, param);

%% Extract solution
if isfield(res, 'sol') && isfield(res.sol, 'itr') && strcmp(res.sol.itr.solsta, 'OPTIMAL')
    U_opt = res.sol.itr.xx;
    info.success = true;
    info.cost = res.sol.itr.pobjval;
    info.solsta = res.sol.itr.solsta;
elseif isfield(res, 'sol') && isfield(res.sol, 'bas') && strcmp(res.sol.bas.solsta, 'OPTIMAL')
    U_opt = res.sol.bas.xx;
    info.success = true;
    info.cost = res.sol.bas.pobjval;
    info.solsta = res.sol.bas.solsta;
else
    U_opt = zeros(m * N, 1);
    info.success = false;
    if isfield(res, 'rcodestr')
        info.message = res.rcodestr;
    else
        info.message = 'MOSEK failed';
    end
end

%% Reshape output
u_opt = reshape(U_opt, m, N);

% Predict states
X_pred = Sx * x0 + Su * U_opt;
x_pred = reshape(X_pred, n, N);

info.res = res;
end
