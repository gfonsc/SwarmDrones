function x_next = quadrotor_dynamics(x, u, params)
%QUADROTOR_DYNAMICS Nonlinear dynamics of quadrotor (Crazyflie)
%
%   State: x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]'  (12x1)
%          Position, velocity, Euler angles (ZYX), angular velocity
%
%   Input: u = [T; tau_x; tau_y; tau_z]  (4x1)
%          Total thrust [N], body torques [Nm]
%
%   Params: struct from crazyflie_params()
%
%   Uses Euler integration (can upgrade to RK4 if needed)

%% Extract parameters
m = params.m;
g = params.g;
J = params.J;
dt = params.dt;

%% Extract state
pos = x(1:3);      % Position [x, y, z]
vel = x(4:6);      % Velocity [vx, vy, vz]
euler = x(7:9);    % Euler angles [phi, theta, psi] (roll, pitch, yaw)
omega = x(10:12);  % Angular velocity [p, q, r]

phi = euler(1);
theta = euler(2);
psi = euler(3);

%% Extract inputs
T = u(1);              % Total thrust
tau = u(2:4);          % Body torques [tau_x, tau_y, tau_z]

%% Rotation matrix (body to world)
R = rotation_matrix_zyx(phi, theta, psi);

%% Translational dynamics
% Thrust in body frame (z-axis)
thrust_body = [0; 0; T];

% Thrust in world frame
thrust_world = R * thrust_body;

% Gravity
gravity = [0; 0; -m * g];

% Acceleration
acc = (thrust_world + gravity) / m;

%% Rotational dynamics
% Euler's equation: J * omega_dot = tau - omega x (J * omega)
omega_dot = J \ (tau - cross(omega, J * omega));

%% Euler angle rates
% Transform angular velocity to Euler rates
T_euler = euler_rate_matrix(phi, theta);
euler_dot = T_euler \ omega;

%% Euler integration
pos_next = pos + dt * vel;
vel_next = vel + dt * acc;
euler_next = euler + dt * euler_dot;
omega_next = omega + dt * omega_dot;

% Normalize angles to [-pi, pi]
euler_next = wrapToPi(euler_next);

%% Pack state
x_next = [pos_next; vel_next; euler_next; omega_next];
end

function R = rotation_matrix_zyx(phi, theta, psi)
%ROTATION_MATRIX_ZYX Rotation matrix from body to world (ZYX convention)
c_phi = cos(phi);   s_phi = sin(phi);
c_theta = cos(theta); s_theta = sin(theta);
c_psi = cos(psi);   s_psi = sin(psi);

R = [c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi;
    s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi;
    -s_theta,      c_theta*s_phi,                     c_theta*c_phi];
end

function T = euler_rate_matrix(phi, theta)
%EULER_RATE_MATRIX Transform angular velocity to Euler rates
c_phi = cos(phi);
s_phi = sin(phi);
c_theta = cos(theta);
t_theta = tan(theta);

T = [1, s_phi*t_theta,  c_phi*t_theta;
    0, c_phi,          -s_phi;
    0, s_phi/c_theta,  c_phi/c_theta];
end
