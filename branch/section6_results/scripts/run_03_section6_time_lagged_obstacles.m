% run_03_section6_time_lagged_obstacles.m
% Implements Scenario C as a visualization of the prediction horizon (time-lagged perspective)
% This does not change the controller; it visualizes the predictive state at a specific snapshot.
addpath('functions');

deepc_file = fullfile('..', 'results', 'iterative_obstacle_results.mat');
mpc_file = fullfile('..', 'mpc_baseline', 'results', 'mpc_obstacle_results.mat');

data_deepc = load(deepc_file);
data_mpc = load(mpc_file);

std_deepc = section6_standardize_result_struct(data_deepc.result.log, 'DeePC', 'Scenario C - Snapshot Visualization', data_deepc.result.scenario);
std_mpc = section6_standardize_result_struct(data_mpc.obstacleResults.iterativeResult.log, 'MPC', 'Scenario C - Snapshot Visualization', data_deepc.result.scenario);

out_dir = fullfile('data');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
save(fullfile(out_dir, 'scenario_C_time_lagged_obstacles_results.mat'), 'std_deepc', 'std_mpc');

fig_dir = fullfile('figures', 'scenario_C_time_lagged_obstacles');

% Pick a time step where constraint is active (e.g. near obstacle)
idx = find(data_deepc.result.log.constraintCount > 0, 1, 'first');
if isempty(idx)
    idx = round(length(std_deepc.t) / 2);
end

fig2d = figure('Name', 'Time-Lagged Snapshot 2D', 'Position', [100, 100, 800, 600]);
hold on;

% Plot full reference
plot(std_deepc.ref.position(:,1), std_deepc.ref.position(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

% Plot past trajectory up to idx
plot(std_deepc.actual.position(1:idx,1), std_deepc.actual.position(1:idx,2), 'b-', 'LineWidth', 2, 'DisplayName', 'DeePC Past');
plot(std_mpc.actual.position(1:idx,1), std_mpc.actual.position(1:idx,2), 'r-', 'LineWidth', 2, 'DisplayName', 'MPC Past');

% Current position
plot(std_deepc.actual.position(idx,1), std_deepc.actual.position(idx,2), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'DeePC Current');
plot(std_mpc.actual.position(idx,1), std_mpc.actual.position(idx,2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'MPC Current');

% Plot predicted horizon if available (DeePC)
if isfield(data_deepc.result.log, 'pred')
    if iscell(data_deepc.result.log.pred) && length(data_deepc.result.log.pred) >= idx
        pred = data_deepc.result.log.pred{idx};
        pred_vec = reshape(pred, 3, [])';
        plot(pred_vec(:,1), pred_vec(:,2), 'b:', 'LineWidth', 1.5, 'DisplayName', 'DeePC Predicted');
    elseif isnumeric(data_deepc.result.log.pred) && size(data_deepc.result.log.pred, 1) >= idx
        pred = data_deepc.result.log.pred(idx, :); % N*3 elements
        pred_vec = reshape(pred, 3, [])';
        plot(pred_vec(:,1), pred_vec(:,2), 'b:', 'LineWidth', 1.5, 'DisplayName', 'DeePC Predicted');
    end
end

% Obstacles
obstacles = std_deepc.obstacles;
for i = 1:size(obstacles, 1)
    obs = obstacles(i,:);
    c = obs(1:2);
    r = obs(5);
    r_safe = r + 0.25;
    th = linspace(0, 2*pi, 50);
    fill(c(1) + r*cos(th), c(2) + r*sin(th), [0.7 0.7 0.7], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(c(1) + r_safe*cos(th), c(2) + r_safe*sin(th), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
fill(NaN, NaN, [0.7 0.7 0.7], 'EdgeColor', 'none', 'DisplayName', 'Obstacle');

axis equal;
xlabel('X [m]'); ylabel('Y [m]');
grid on; box on; legend('Location', 'best');
section6_apply_plot_style();
section6_export_figure(fig2d, fullfile(fig_dir, 'C_2d_trajectory_time_lagged_obstacles'));

% Duplicate as 3D
fig3d = figure('Name', 'Time-Lagged Snapshot 3D', 'Position', [100, 100, 800, 600]);
hold on;
plot3(std_deepc.ref.position(:,1), std_deepc.ref.position(:,2), std_deepc.ref.position(:,3), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot3(std_deepc.actual.position(1:idx,1), std_deepc.actual.position(1:idx,2), std_deepc.actual.position(1:idx,3), 'b-', 'LineWidth', 2, 'DisplayName', 'DeePC Past');
plot3(std_mpc.actual.position(1:idx,1), std_mpc.actual.position(1:idx,2), std_mpc.actual.position(1:idx,3), 'r-', 'LineWidth', 2, 'DisplayName', 'MPC Past');
plot3(std_deepc.actual.position(idx,1), std_deepc.actual.position(idx,2), std_deepc.actual.position(idx,3), 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'DeePC Current');
if exist('pred_vec', 'var')
    plot3(pred_vec(:,1), pred_vec(:,2), pred_vec(:,3), 'b:', 'LineWidth', 1.5, 'DisplayName', 'DeePC Predicted');
end
view(3); grid on; box on;
section6_apply_plot_style();
section6_export_figure(fig3d, fullfile(fig_dir, 'C_3d_trajectory_time_lagged_obstacles'));

% The prompt requires all standard plots for C as well:
figxyz = section6_plot_xyz(std_deepc.t, std_deepc.ref.position, std_deepc.actual.position, std_mpc.actual.position);
section6_export_figure(figxyz, fullfile(fig_dir, 'C_xyz_tracking'));

fig_euler = section6_plot_euler_angles(std_mpc.t, [], std_deepc.actual.euler, std_mpc.actual.euler);
section6_export_figure(fig_euler, fullfile(fig_dir, 'C_euler_angles'));

fig_motor = section6_plot_motor_efforts(std_deepc.t, std_deepc.control.motors, std_mpc.control.motors);
section6_export_figure(fig_motor, fullfile(fig_dir, 'C_motor_efforts'));

fig_state = section6_plot_state_machine(std_deepc.t, std_deepc.state_machine.state_id, std_mpc.state_machine.state_id);
section6_export_figure(fig_state, fullfile(fig_dir, 'C_state_machine'));

fig_margin = section6_plot_safety_margin(std_deepc.t, std_deepc.safety.h, std_mpc.safety.h);
section6_export_figure(fig_margin, fullfile(fig_dir, 'C_safety_margin'));

close all;
disp('Scenario C completed.');
