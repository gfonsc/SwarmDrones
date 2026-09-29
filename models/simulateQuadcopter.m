function state_next = simulateQuadcopter(state_curr, u, dt, params)
% SIMULATEQUADCOPTER 
%   Simulates nonlinear dynamics of a quadrotor for a single timestep dt.
%   Based strictly on Coulson thesis Eq 5.1a and 5.1b.
%
%   INPUTS:
%       state_curr - Current 12D state:
%                    [px, py, pz, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]'
%       u          - Control inputs: [Thrust, tau_x, tau_y, tau_z]'
%       dt         - Timestep
%       params     - Physics parameters config struct (optional)
%
%   OUTPUT:
%       state_next - Next 12D state after dt

    if nargin < 4
        % Default Crazyflie ~45g physics using IDSIA params
        params.mass = 0.045; 
        params.g = 9.81;
        % Inertia approx 
        params.J = diag([2.3951e-5, 2.3951e-5, 3.2347e-5]);
    end

    % Unpack state
    pos = state_curr(1:3);
    vel = state_curr(4:6);
    euler = state_curr(7:9); % [gamma, beta, alpha] = [roll, pitch, yaw]
    omega = state_curr(10:12);
    
    % Unpack input
    f_tot = u(1);
    tau = u(2:4);
    
    phi = euler(1); theta = euler(2); psi = euler(3);
    
    % Acceleration (Z-Y-X Euler intrinsic conversion to inertial frame)
    % R matrix maps body to inertial.
    % Just using the direct formulation from Coulson 5.1a:
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_total = R_z * R_y * R_x;
    
    % Thrust is applied in +Z body direction.
    thrust_body = [0; 0; f_tot];
    gravity_inertial = [0; 0; -params.mass * params.g]; % Z upwards
    
    accel = (1/params.mass) * (R_total * thrust_body + gravity_inertial);
    
    % Angular acceleration (Eq 5.1b)
    alpha = params.J \ (tau - cross(omega, params.J * omega));
    
    % Euler derivatives (mapping body rates to euler rate)
    % Simplified approximation for near-hover (small angles): euler_dot ~= omega
    % Exact:
    C = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi), -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    euler_dot = C * omega;

    % Euler integration
    pos_next = pos + vel * dt;
    vel_next = vel + accel * dt;
    euler_next = euler + euler_dot * dt;
    omega_next = omega + alpha * dt;
    
    state_next = [pos_next; vel_next; euler_next; omega_next];
end
