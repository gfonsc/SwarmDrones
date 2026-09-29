function [mpc_controller, plant] = design_mpc_quadrotor(params)
%DESIGN_MPC_QUADROTOR Design MPC controller for Crazyflie quadrotor
%
%   Creates a Model Predictive Controller for trajectory tracking
%   based on linearized dynamics around hover.
%
%   Usage:
%       params = crazyflie_params();
%       [mpc_controller, plant] = design_mpc_quadrotor(params);
%
%   Outputs:
%       mpc_controller - MPC controller object
%       plant          - Discretized state-space model
%
%   Reference: Benotsmane et al. "MPC for Autonomous Quadrotor"

fprintf('=== Designing MPC Controller for Crazyflie ===\n\n');

%% Get linearized model
fprintf('Step 1: Linearizing quadrotor dynamics...\n');
[A, B, C, D] = linearize_quadrotor(params);

%% Create continuous-time state-space model
sys_c = ss(A, B, C, D);

%% Discretize
Ts = params.dt;  % Sample time (100 Hz)
fprintf('Step 2: Discretizing (Ts = %.3f s, %.0f Hz)...\n', Ts, 1/Ts);

plant = c2d(sys_c, Ts, 'zoh');

%% MPC parameters (from the article, adapted for 100 Hz)
fprintf('Step 3: Configuring MPC parameters...\n');

% Horizons
P = 20;   % Prediction horizon (steps)
M = 3;    % Control horizon (steps)

fprintf('  Prediction horizon: %d steps (%.2f s)\n', P, P*Ts);
fprintf('  Control horizon: %d steps\n', M);

%% Create MPC controller
mpc_controller = mpc(plant, Ts, P, M);

%% Output weights (position + orientation tracking)
% Higher weight = higher priority
% Outputs: [px, py, pz, phi, theta, psi]
mpc_controller.Weights.OutputVariables = [1, 1, 1, 0.1, 0.1, 0.1];

%% Input weights (minimize control effort)
% Inputs: [dT, tau_x, tau_y, tau_z]
mpc_controller.Weights.ManipulatedVariables = [0.1, 0.1, 0.1, 0.1];
mpc_controller.Weights.ManipulatedVariablesRate = [0.01, 0.01, 0.01, 0.01];

%% Input constraints
% Based on Crazyflie capabilities
% dT: deviation from hover thrust (mg = 0.45 N)
% Can provide about 80% extra at max
dT_max = params.T_max - params.m * params.g;  % Extra thrust capacity

% Torques (approximate max from motor specs)
tau_max = max(params.max_torque);

fprintf('Step 4: Setting constraints...\n');
fprintf('  Thrust deviation: [%.3f, %.3f] N\n', -params.m * params.g * 0.5, dT_max);
fprintf('  Torques (each): [%.4f, %.4f] Nm\n', -tau_max, tau_max);

% MV constraints
mpc_controller.MV(1).Min = -params.m * params.g * 0.5;  % Min thrust deviation
mpc_controller.MV(1).Max = dT_max;

for i = 2:4
    mpc_controller.MV(i).Min = -tau_max;
    mpc_controller.MV(i).Max = tau_max;
end

%% Output constraints (soft, for safety)
% Allow position to move freely, but constrain angles
angle_max = deg2rad(30);  % Max roll/pitch
yaw_max = deg2rad(180);   % Allow full rotation

mpc_controller.OV(4).Min = -angle_max;  % phi
mpc_controller.OV(4).Max = angle_max;
mpc_controller.OV(5).Min = -angle_max;  % theta
mpc_controller.OV(5).Max = angle_max;
mpc_controller.OV(6).Min = -yaw_max;    % psi
mpc_controller.OV(6).Max = yaw_max;

%% Display summary
fprintf('\n=== MPC Controller Designed ===\n');
fprintf('Plant: 12 states, 4 inputs, 6 outputs\n');
fprintf('Sample time: %.0f Hz\n', 1/Ts);
fprintf('Horizons: P=%d, M=%d\n', P, M);

end
