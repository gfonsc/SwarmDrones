%% test_mpc_quadrotor.m
% Test MPC controller on quadrotor trajectory tracking
%
% This script:
%   1. Designs MPC controller
%   2. Simulates trajectory tracking
%   3. Compares with DeePC if available

clear; clc; close all;

fprintf('========================================\n');
fprintf('MPC Quadrotor Trajectory Tracking Test\n');
fprintf('========================================\n\n');

%% Step 1: Get parameters and design MPC
params = crazyflie_params();
[mpc_controller, plant] = design_mpc_quadrotor(params);

%% Step 2: Generate reference trajectory
fprintf('\nStep 2: Generating test trajectory...\n');

Ts = params.dt;
T_sim = 20;  % Simulation time [s]
t = 0:Ts:T_sim;
n_steps = length(t);

% Sinusoidal trajectory (similar to zigzag test)
x_ref = linspace(0, 1, n_steps)';
y_ref = 0.3 * sin(4 * pi * t / T_sim)';
z_ref = linspace(0.5, 1.5, n_steps)';

% Angles: hover (zero)
phi_ref = zeros(n_steps, 1);
theta_ref = zeros(n_steps, 1);
psi_ref = zeros(n_steps, 1);

% Reference for MPC (6 outputs)
ref = [x_ref, y_ref, z_ref, phi_ref, theta_ref, psi_ref];

fprintf('Trajectory: %.1f s, %d steps\n', T_sim, n_steps);
fprintf('X: [%.2f, %.2f] m\n', min(x_ref), max(x_ref));
fprintf('Y: [%.2f, %.2f] m\n', min(y_ref), max(y_ref));
fprintf('Z: [%.2f, %.2f] m\n', min(z_ref), max(z_ref));

%% Step 3: Simulate MPC
fprintf('\nStep 3: Running MPC simulation...\n');

% Initial state (hover at start of trajectory)
x0 = zeros(12, 1);
x0(1) = x_ref(1);  % Start at reference position
x0(2) = y_ref(1);
x0(3) = z_ref(1);

% Storage
x_mpc = zeros(n_steps, 12);
u_mpc = zeros(n_steps, 4);
solve_times = zeros(n_steps, 1);

x_mpc(1, :) = x0';

% MPC state
mpc_state = mpcstate(mpc_controller);

tic;
for k = 1:n_steps-1
    % Current state
    x_curr = x_mpc(k, :)';

    % Current measurement (outputs)
    y_curr = plant.C * x_curr;

    % Compute MPC control
    t_solve = tic;
    u = mpcmove(mpc_controller, mpc_state, y_curr, ref(k:min(k+19, n_steps), :));
    solve_times(k) = toc(t_solve);

    % Store control
    u_mpc(k, :) = u';

    % Simulate one step (using discrete plant)
    x_next = plant.A * x_curr + plant.B * u;
    x_mpc(k+1, :) = x_next';

    % Progress
    if mod(k, 100) == 0
        fprintf('  Step %d/%d (%.1f ms/step)\n', k, n_steps, mean(solve_times(1:k))*1000);
    end
end
total_time = toc;

fprintf('Simulation complete in %.2f s\n', total_time);
fprintf('Average solve time: %.2f ms\n', mean(solve_times)*1000);

%% Step 4: Compute errors
pos_mpc = x_mpc(:, 1:3);
pos_ref = [x_ref, y_ref, z_ref];

tracking_error = vecnorm(pos_ref - pos_mpc, 2, 2);

fprintf('\n=== MPC Performance Metrics ===\n');
fprintf('Mean tracking error: %.2f mm\n', mean(tracking_error) * 1000);
fprintf('Max tracking error:  %.2f mm\n', max(tracking_error) * 1000);
fprintf('RMS error:           %.2f mm\n', rms(tracking_error) * 1000);

%% Step 5: Plot results
fprintf('\nStep 5: Generating plots...\n');

fig = figure('Name', 'MPC Quadrotor Control', 'Position', [50, 50, 1200, 800], 'Color', 'w');

% 3D Trajectory
subplot(2, 2, [1, 3]);
plot3(x_ref, y_ref, z_ref, 'b-', 'LineWidth', 3, 'DisplayName', 'Reference');
hold on;
plot3(pos_mpc(:,1), pos_mpc(:,2), pos_mpc(:,3), 'r--', 'LineWidth', 2.5, 'DisplayName', 'MPC');
plot3(x_ref(1), y_ref(1), z_ref(1), 'go', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerFaceColor', 'g');
plot3(x_ref(end), y_ref(end), z_ref(end), 'rs', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerFaceColor', 'r');
xlabel('X (m)', 'FontSize', 14);
ylabel('Y (m)', 'FontSize', 14);
zlabel('Z (m)', 'FontSize', 14);
title('3D Trajectory', 'FontSize', 16);
legend('Location', 'best', 'FontSize', 12);
grid on;
view(45, 25);

% Position time series
subplot(2, 2, 2);
plot(t, x_ref, 'b-', 'LineWidth', 2); hold on;
plot(t, pos_mpc(:,1), 'r--', 'LineWidth', 2);
plot(t, y_ref, 'b-', 'LineWidth', 2);
plot(t, pos_mpc(:,2), 'r--', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Position (m)', 'FontSize', 12);
title('Position Tracking (X, Y)', 'FontSize', 14);
legend('Ref', 'MPC', 'Location', 'best');
grid on;

% Tracking error
subplot(2, 2, 4);
plot(t, tracking_error * 1000, 'k-', 'LineWidth', 1.5);
hold on;
yline(mean(tracking_error)*1000, 'r--', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Error (mm)', 'FontSize', 12);
title(sprintf('Tracking Error (Mean: %.2f mm)', mean(tracking_error)*1000), 'FontSize', 14);
grid on;

sgtitle('MPC Controller - Crazyflie Model', 'FontSize', 18, 'FontWeight', 'bold');

drawnow;
saveas(fig, 'mpc_trajectory_results.png');
fprintf('Saved: mpc_trajectory_results.png\n');

%% Save results for comparison
mpc_results.x_mpc = x_mpc;
mpc_results.u_mpc = u_mpc;
mpc_results.tracking_error = tracking_error;
mpc_results.solve_times = solve_times;
mpc_results.ref = ref;
mpc_results.t = t;

save('mpc_results.mat', 'mpc_results');
fprintf('Results saved to mpc_results.mat\n');

fprintf('\n=== MPC Test Complete ===\n');
