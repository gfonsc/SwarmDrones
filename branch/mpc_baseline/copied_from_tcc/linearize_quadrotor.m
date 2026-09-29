function [A, B, C, D] = linearize_quadrotor(params)
%LINEARIZE_QUADROTOR Linearize quadrotor dynamics around hover
%
%   Computes state-space matrices A, B, C, D for the linearized
%   quadrotor model around hover equilibrium.
%
%   State: x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]'  (12x1)
%   Input: u = [T - mg; tau_x; tau_y; tau_z]  (4x1) - deviation from hover
%   Output: y = [px, py, pz, phi, theta, psi]'  (6x1)
%
%   Usage:
%       params = crazyflie_params();
%       [A, B, C, D] = linearize_quadrotor(params);
%
%   Reference: Benotsmane et al. "MPC for Autonomous Quadrotor"

%% Extract parameters
m = params.m;
g = params.g;
Jxx = params.Jxx;
Jyy = params.Jyy;
Jzz = params.Jzz;

%% State indices
% x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
%      1    2   3   4   5   6    7     8     9  10  11  12

n = 12;  % Number of states
m_u = 4; % Number of inputs
p = 6;   % Number of outputs

%% System matrix A (12x12)
% Linearized around hover: phi=0, theta=0, all velocities = 0
A = zeros(n, n);

% Position derivatives = velocity
A(1, 4) = 1;  % dx/dvx
A(2, 5) = 1;  % dy/dvy
A(3, 6) = 1;  % dz/dvz

% Velocity dynamics (linearized)
% At hover: thrust = mg
% Small angle: vx_dot ≈ g*theta, vy_dot ≈ -g*phi
A(4, 8) = g;   % dvx/dtheta (pitch forward → accelerate x)
A(5, 7) = -g;  % dvy/dphi (roll right → accelerate -y)

% Euler angle derivatives = angular velocity (small angle)
A(7, 10) = 1;  % dphi/dp
A(8, 11) = 1;  % dtheta/dq
A(9, 12) = 1;  % dpsi/dr

% Angular velocity dynamics are decoupled at hover (J*omega_dot = tau)
% No cross-coupling terms in linearized model

%% Input matrix B (12x4)
% u = [dT, tau_x, tau_y, tau_z]
% dT is deviation from hover thrust
B = zeros(n, m_u);

% Thrust affects z-acceleration
B(6, 1) = 1 / m;  % dvz/dT

% Torques affect angular acceleration
B(10, 2) = 1 / Jxx;  % dp/dtau_x
B(11, 3) = 1 / Jyy;  % dq/dtau_y
B(12, 4) = 1 / Jzz;  % dr/dtau_z

%% Output matrix C (6x12)
% y = [px, py, pz, phi, theta, psi]
C = zeros(p, n);
C(1, 1) = 1;  % px
C(2, 2) = 1;  % py
C(3, 3) = 1;  % pz
C(4, 7) = 1;  % phi
C(5, 8) = 1;  % theta
C(6, 9) = 1;  % psi

%% Feedthrough matrix D (6x4)
D = zeros(p, m_u);

%% Display info
if nargout == 0
    fprintf('Linearized Quadrotor Model:\n');
    fprintf('  States: 12 (pos, vel, euler, omega)\n');
    fprintf('  Inputs: 4 (dT, tau_x, tau_y, tau_z)\n');
    fprintf('  Outputs: 6 (pos, euler)\n');
    fprintf('\nA matrix eigenvalues:\n');
    disp(eig(A)');

    % Check controllability
    Co = ctrb(A, B);
    fprintf('\nControllability rank: %d (of %d)\n', rank(Co), n);

    % Check observability
    Ob = obsv(A, C);
    fprintf('Observability rank: %d (of %d)\n', rank(Ob), n);
end
end
