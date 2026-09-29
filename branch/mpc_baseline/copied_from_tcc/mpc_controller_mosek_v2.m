function [u_opt, x_pred, info] = mpc_controller_mosek_v2(A, B, C, x0, ref, params)
%MPC_CONTROLLER_MOSEK_V2 Improved MPC with MOSEK solver
%
%   Improvements over v1:
%   - Full state tracking (position + velocity)
%   - Terminal cost for stability
%   - Rate penalty on control changes
%   - Better numerical conditioning
%
%   Inputs:
%       A, B, C - Discrete state-space matrices
%       x0      - Current state (n x 1)
%       ref     - Reference trajectory (struct with .pos (3xN) and .vel (3xN))
%       params  - Struct with MPC parameters:
%                   .N        - Prediction horizon
%                   .Q_pos    - Position weight (3x3)
%                   .Q_vel    - Velocity weight (3x3)
%                   .Q_ang    - Angle weight (3x3)
%                   .R        - Input weight (4x4)
%                   .R_delta  - Input rate weight (4x4)
%                   .P_term   - Terminal cost multiplier
%                   .u_min, .u_max - Input bounds (4x1)
%                   .u_prev   - Previous input (for rate penalty)
%
%   Outputs:
%       u_opt  - Optimal input sequence (m x N)
%       x_pred - Predicted state trajectory (n x N+1)
%       info   - Solver information

%% Dimensions
n = size(A, 1);  % 12 states
m = size(B, 2);  % 4 inputs
N = params.N;

%% Extract weights
Q_pos = params.Q_pos;    % Position tracking (3x3)
Q_vel = params.Q_vel;    % Velocity tracking (3x3)
Q_ang = params.Q_ang;    % Angle tracking (3x3)
R = params.R;            % Input penalty (4x4)
R_delta = params.R_delta; % Input rate penalty (4x4)
P_term = params.P_term;  % Terminal cost multiplier

u_min = params.u_min;
u_max = params.u_max;
u_prev = params.u_prev;

%% Build full state weight matrix
% State: [px,py,pz, vx,vy,vz, phi,theta,psi, p,q,r]
% We want to track position (1-3), velocity (4-6), and keep angles small (7-9)
Q = blkdiag(Q_pos, Q_vel, Q_ang, zeros(3));  % 12x12

%% Build prediction matrices
% X = Sx * x0 + Su * U where X = [x_1; ...; x_N], U = [u_0; ...; u_{N-1}]

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

%% Build reference state trajectory
% From position reference, we can compute velocity reference as finite difference
pos_ref = ref.pos;  % 3 x N
if isfield(ref, 'vel')
    vel_ref = ref.vel;  % 3 x N
else
    vel_ref = zeros(3, N);  % Assume zero velocity reference
end

% Extend if needed
if size(pos_ref, 2) < N
    pos_ref = [pos_ref, repmat(pos_ref(:,end), 1, N - size(pos_ref, 2))];
    vel_ref = [vel_ref, repmat(vel_ref(:,end), 1, N - size(vel_ref, 2))];
end

% Build full state reference (angles = 0, angular rates = 0)
x_ref = zeros(n, N);
x_ref(1:3, :) = pos_ref;
x_ref(4:6, :) = vel_ref;
% angles (7:9) and rates (10:12) remain zero

x_ref_vec = reshape(x_ref, [], 1);  % nN x 1

%% Build cost matrices
% Stage cost: sum_k (x_k - r_k)' Q (x_k - r_k) + u_k' R u_k

Q_bar = kron(eye(N), Q);

% Terminal cost (P = P_term * Q for last step)
Q_terminal = P_term * Q;
Q_bar((N-1)*n+1:N*n, (N-1)*n+1:N*n) = Q_terminal;

% Input penalty
R_bar = kron(eye(N), R);

%% Rate penalty: sum_k (u_k - u_{k-1})' R_delta (u_k - u_{k-1})
% u_0 - u_prev, u_1 - u_0, etc.

% Build difference matrix D such that delta_U = D * U + d
% where delta_U = [u_0 - u_prev; u_1 - u_0; ...]
D = zeros(m * N, m * N);
for k = 1:N
    D((k-1)*m+1:k*m, (k-1)*m+1:k*m) = eye(m);
    if k > 1
        D((k-1)*m+1:k*m, (k-2)*m+1:(k-1)*m) = -eye(m);
    end
end

% d = [u_prev; 0; 0; ...]
d = zeros(m * N, 1);
d(1:m) = -u_prev;  % So delta_0 = u_0 - u_prev

R_delta_bar = kron(eye(N), R_delta);

%% Full Hessian
% Cost = (X - X_ref)' Q_bar (X - X_ref) + U' R_bar U + (DU + d)' R_delta_bar (DU + d)
% X = Sx*x0 + Su*U
% So: (Sx*x0 + Su*U - X_ref)' Q_bar (...) + U' R_bar U + (DU + d)' R_delta_bar (DU + d)

% Expanding:
% = U' (Su' Q_bar Su + R_bar + D' R_delta_bar D) U
%   + 2 (Sx*x0 - X_ref)' Q_bar Su U
%   + 2 d' R_delta_bar D U
%   + const

H = Su' * Q_bar * Su + R_bar + D' * R_delta_bar * D;
H = (H + H') / 2;  % Symmetrize
H = H + 1e-6 * eye(size(H));  % Regularization

f = 2 * Su' * Q_bar * (Sx * x0 - x_ref_vec) + 2 * D' * R_delta_bar * d;

%% Bounds
lb = repmat(u_min, N, 1);
ub = repmat(u_max, N, 1);

%% Solve with MOSEK
prob = struct();
[prob.qosubi, prob.qosubj, prob.qoval] = find(tril(H));
prob.c = f;
prob.blx = lb;
prob.bux = ub;
prob.a = sparse(0, m * N);
prob.blc = [];
prob.buc = [];

param.MSK_IPAR_LOG = 0;
[~, res] = mosekopt('minimize echo(0)', prob, param);

%% Extract solution
if isfield(res, 'sol') && isfield(res.sol, 'itr') && strcmp(res.sol.itr.solsta, 'OPTIMAL')
    U_opt = res.sol.itr.xx;
    info.success = true;
    info.cost = res.sol.itr.pobjval;
elseif isfield(res, 'sol') && isfield(res.sol, 'bas') && strcmp(res.sol.bas.solsta, 'OPTIMAL')
    U_opt = res.sol.bas.xx;
    info.success = true;
    info.cost = res.sol.bas.pobjval;
else
    U_opt = zeros(m * N, 1);
    info.success = false;
    info.message = 'MOSEK failed';
end

%% Output
u_opt = reshape(U_opt, m, N);

% Predict full trajectory
X_pred = Sx * x0 + Su * U_opt;
x_pred = [x0, reshape(X_pred, n, N)];  % Include initial state

info.res = res;
end
