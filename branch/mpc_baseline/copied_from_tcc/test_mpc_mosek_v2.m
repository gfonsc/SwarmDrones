%% test_mpc_mosek_v2.m
% Test improved MPC controller (v2) with MOSEK solver
%
% Improvements:
%   - Velocity tracking (not just position)
%   - Terminal cost for stability
%   - Input rate penalty for smoother control
%   - Better weight tuning

clear; clc; close all;

fprintf('========================================\n');
fprintf('MPC-MOSEK v2 (Improved)\n');
fprintf('========================================\n\n');

%% Step 1: Setup model
fprintf('Step 1: Setting up quadrotor model...\n');
phys_params = crazyflie_params();
[A_c, B_c, C, D] = linearize_quadrotor(phys_params);

Ts = phys_params.dt;
sys_d = c2d(ss(A_c, B_c, C, D), Ts, 'zoh');
A = sys_d.A;
B = sys_d.B;

fprintf('  Model: 12 states, 4 inputs\n');
fprintf('  Sample time: %.3f s\n', Ts);

%% Step 2: Configure MPC parameters
fprintf('\nStep 2: Configuring improved MPC...\n');

mpc_params = struct();
mpc_params.N = 30;  % Longer horizon for better performance

% Weights - much higher for position tracking
mpc_params.Q_pos = 100 * eye(3);   % Position tracking (HIGH)
mpc_params.Q_vel = 10 * eye(3);    % Velocity tracking (helps damping)
mpc_params.Q_ang = 1 * eye(3);     % Keep angles small

% Input weights
mpc_params.R = 0.01 * eye(4);      % Allow aggressive control
mpc_params.R_delta = 0.1 * eye(4); % Smooth control changes

% Terminal cost (stabilizes)
mpc_params.P_term = 10;

% Constraints
dT_max = phys_params.T_max - phys_params.m * phys_params.g;
tau_max = max(phys_params.max_torque);
mpc_params.u_min = [-phys_params.m * phys_params.g * 0.5; -tau_max; -tau_max; -tau_max];
mpc_params.u_max = [dT_max; tau_max; tau_max; tau_max];

% Initial previous control
mpc_params.u_prev = zeros(4, 1);

fprintf('  Horizon: %d steps (%.2f s)\n', mpc_params.N, mpc_params.N * Ts);
fprintf('  Q_pos scale: %.0f, Q_vel scale: %.0f\n', mpc_params.Q_pos(1,1), mpc_params.Q_vel(1,1));

%% Step 3: Generate trajectory (IDSIA-style square at multiple altitudes)
fprintf('\nStep 3: Generating 3D trajectory...\n');

%% Generate trajectory
traj_type = 'square'; % 'square', 'circular', 'ellipse', 'lemniscate', 'racing'
addpath('..'); % To find generate_trajectory.m
T_sim = 100;
t = 0:Ts:T_sim;
n_steps = length(t);

ref_pos = generate_trajectory(traj_type, t, phys_params);
x_ref = ref_pos(1,:);
y_ref = ref_pos(2,:);
z_ref = ref_pos(3,:);

% Compute velocity reference
vx_ref = gradient(x_ref, Ts);
vy_ref = gradient(y_ref, Ts);
vz_ref = gradient(z_ref, Ts);

ref.pos = [x_ref; y_ref; z_ref];
ref.vel = [vx_ref; vy_ref; vz_ref];

fprintf('  Duration: %.1f s, %d steps\n', T_sim, n_steps);
fprintf('  Trajectory: %s\n', traj_type);

%% Step 4: Simulate
fprintf('\nStep 4: Running simulation...\n');

n = size(A, 1);
m = size(B, 2);

x0 = zeros(n, 1);
x0(1:3) = ref.pos(:, 1);

x_mpc = zeros(n, n_steps);
u_mpc = zeros(m, n_steps);
solve_times = zeros(1, n_steps);
x_mpc(:, 1) = x0;

tic;
for k = 1:n_steps-1
    % Get reference for horizon
    ref_start = k;
    ref_end = min(k + mpc_params.N - 1, n_steps);

    ref_horizon.pos = ref.pos(:, ref_start:ref_end);
    ref_horizon.vel = ref.vel(:, ref_start:ref_end);

    % Extend if too short
    if size(ref_horizon.pos, 2) < mpc_params.N
        n_pad = mpc_params.N - size(ref_horizon.pos, 2);
        ref_horizon.pos = [ref_horizon.pos, repmat(ref_horizon.pos(:,end), 1, n_pad)];
        ref_horizon.vel = [ref_horizon.vel, zeros(3, n_pad)];  % Zero velocity at end
    end

    % Update previous control for rate penalty
    if k > 1
        mpc_params.u_prev = u_mpc(:, k-1);
    end

    % Solve MPC
    t_solve = tic;
    [u_opt, ~, info] = mpc_controller_mosek_v2(A, B, C, x_mpc(:,k), ref_horizon, mpc_params);
    solve_times(k) = toc(t_solve);

    if info.success
        u_mpc(:, k) = u_opt(:, 1);
    else
        u_mpc(:, k) = zeros(m, 1);
    end

    % Simulate
    x_mpc(:, k+1) = A * x_mpc(:, k) + B * u_mpc(:, k);

    % Progress
    if mod(k, 100) == 0
        fprintf('  Step %d/%d (%.2f ms/step)\n', k, n_steps, mean(solve_times(1:k))*1000);
    end
end
total_time = toc;

fprintf('Complete in %.2f s (%.2f ms/step)\n', total_time, mean(solve_times)*1000);

%% Step 5: Results
pos_mpc = x_mpc(1:3, :)';
pos_ref = ref.pos';

tracking_error = vecnorm(pos_ref - pos_mpc, 2, 2);

fprintf('\n=== MPC-MOSEK v2 Performance ===\n');
fprintf('Mean error: %.2f mm\n', mean(tracking_error) * 1000);
fprintf('Max error:  %.2f mm\n', max(tracking_error) * 1000);
fprintf('RMS error:  %.2f mm\n', rms(tracking_error) * 1000);

%% Step 6: Plot
fprintf('\nGenerating plots...\n');

fig = figure('Name', 'MPC-MOSEK v2', 'Position', [50, 50, 1400, 500], 'Color', 'w');

% 3D
subplot(1, 3, 1);
plot3(ref.pos(1,:), ref.pos(2,:), ref.pos(3,:), 'b-', 'LineWidth', 3);
hold on;
plot3(pos_mpc(:,1), pos_mpc(:,2), pos_mpc(:,3), 'r--', 'LineWidth', 2.5);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory', 'FontSize', 14);
legend('Reference', 'MPC v2', 'Location', 'best');
grid on; view(45, 25);

% Time series
subplot(1, 3, 2);
plot(t, ref.pos(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t, pos_mpc(:,1), 'r--', 'LineWidth', 2);
plot(t, ref.pos(2,:), 'b-', 'LineWidth', 2);
plot(t, pos_mpc(:,2), 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position (m)');
title('X, Y Tracking', 'FontSize', 14);
grid on;

% Error
subplot(1, 3, 3);
plot(t, tracking_error * 1000, 'k-', 'LineWidth', 1.5);
hold on;
yline(mean(tracking_error)*1000, 'r--', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Error (mm)');
title(sprintf('Error (Mean: %.2f mm)', mean(tracking_error)*1000), 'FontSize', 14);
grid on;

sgtitle('MPC-MOSEK v2 (Improved)', 'FontSize', 18, 'FontWeight', 'bold');

drawnow;
saveas(fig, 'mpc_mosek_v2_results.png');
fprintf('Saved: mpc_mosek_v2_results.png\n');

%% Save (including full state with Euler angles)
% x_mpc state: [px,py,pz, vx,vy,vz, phi,theta,psi, p,q,r]
euler_mpc = x_mpc(7:9, :);  % [roll; pitch; yaw] in radians (3 x N)
vel_mpc = x_mpc(4:6, :);    % [vx; vy; vz] (3 x N)
pos_mpc_3xN = x_mpc(1:3, :);  % Position as 3 x N for visualization

save('mpc_mosek_v2_results.mat', 'pos_mpc', 'pos_mpc_3xN', 'euler_mpc', 'vel_mpc', ...
    'u_mpc', 'tracking_error', 'solve_times', 't', 'x_mpc', 'ref');

fprintf('Saved: mpc_mosek_v2_results.mat\n');
fprintf('  Contains: x_mpc (12x%d), euler_mpc (3x%d), pos_mpc (%dx3)\n', ...
    size(x_mpc, 2), size(euler_mpc, 2), size(pos_mpc, 1));

fprintf('\n=== Test Complete ===\n');
