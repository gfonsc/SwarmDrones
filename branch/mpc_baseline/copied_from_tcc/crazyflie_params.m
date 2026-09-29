function params = crazyflie_params()
%CRAZYFLIE_PARAMS Physical parameters for Crazyflie 2.1 Brushless
%
%   Based on IDSIA nanodrone-sysid-benchmark
%
%   Usage:
%       params = crazyflie_params();
%
%   Reference: IDSIA benchmark paper (2512.14450v1)

%% Mass and geometry
params.m = 0.045;              % Mass [kg]
params.g = 9.81;               % Gravity [m/s^2]
params.arm_length = 0.0353;    % Arm length [m] (center to rotor)

%% Motor coefficients
params.Kt = 3.72e-08;          % Thrust coefficient [N/(rad/s)^2]
params.Kc = 7.74e-12;          % Moment coefficient [Nm/(rad/s)^2]

%% Thrust to weight ratio
params.thrust_to_weight = 1.8; % From IDSIA paper
params.T_max = params.thrust_to_weight * params.m * params.g;  % Max thrust [N]

%% Inertia tensor (diagonal, [kg*m^2])
% Approximate values for Crazyflie 2.1
params.Jxx = 1.4e-5;
params.Jyy = 1.4e-5;
params.Jzz = 2.2e-5;

params.J = diag([params.Jxx, params.Jyy, params.Jzz]);
params.J_inv = inv(params.J);

%% Max torques
% Computed from motor specs
params.max_torque_xy = params.Kt * params.arm_length * 4 * (1200)^2 * 0.5;  % Approx
params.max_torque_z = params.Kc * 4 * (1200)^2 * 0.5;
params.max_torque = [params.max_torque_xy; params.max_torque_xy; params.max_torque_z];

%% Sample time
params.dt = 0.01;  % 100 Hz (matches IDSIA data)

%% Display
if nargout == 0
    fprintf('Crazyflie 2.1 Brushless Parameters:\n');
    fprintf('  Mass: %.4f kg\n', params.m);
    fprintf('  Arm length: %.4f m\n', params.arm_length);
    fprintf('  Kt: %.2e N/(rad/s)^2\n', params.Kt);
    fprintf('  Kc: %.2e Nm/(rad/s)^2\n', params.Kc);
    fprintf('  T_max: %.3f N\n', params.T_max);
    fprintf('  J: diag([%.2e, %.2e, %.2e]) kg*m^2\n', params.Jxx, params.Jyy, params.Jzz);
end
end
