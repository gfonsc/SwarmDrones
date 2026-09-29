% run_02_section6_with_obstacles.m
addpath('functions');

deepc_file = fullfile('..', 'results', 'iterative_obstacle_results.mat');
mpc_file = fullfile('..', 'mpc_baseline', 'results', 'mpc_obstacle_results.mat');

data_deepc = load(deepc_file);
data_mpc = load(mpc_file);

% Standardize
std_deepc = section6_standardize_result_struct(data_deepc.result.log, 'DeePC', 'Scenario B - With Obstacles', data_deepc.result.scenario);
std_mpc = section6_standardize_result_struct(data_mpc.obstacleResults.iterativeResult.log, 'MPC', 'Scenario B - With Obstacles', data_deepc.result.scenario);

% Save standardized data
out_dir = fullfile('data');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
save(fullfile(out_dir, 'scenario_B_with_obstacles_results.mat'), 'std_deepc', 'std_mpc');

% Plot
fig_dir = fullfile('figures', 'scenario_B_with_obstacles');

% B_2d_trajectory
fig2d = section6_plot_2d_trajectory(std_deepc.ref.position, std_deepc.actual.position, std_mpc.actual.position, std_deepc.obstacles, std_deepc.walls);
section6_export_figure(fig2d, fullfile(fig_dir, 'B_2d_trajectory_obstacles'));

% B_3d_trajectory
fig3d = section6_plot_3d_trajectory(std_deepc.ref.position, std_deepc.actual.position, std_mpc.actual.position, std_deepc.obstacles, std_deepc.walls);
section6_export_figure(fig3d, fullfile(fig_dir, 'B_3d_trajectory_obstacles'));

% B_xyz_tracking
figxyz = section6_plot_xyz(std_deepc.t, std_deepc.ref.position, std_deepc.actual.position, std_mpc.actual.position);
section6_export_figure(figxyz, fullfile(fig_dir, 'B_xyz_tracking'));

% B_euler_angles
fig_euler = section6_plot_euler_angles(std_mpc.t, [], std_deepc.actual.euler, std_mpc.actual.euler);
section6_export_figure(fig_euler, fullfile(fig_dir, 'B_euler_angles'));

% B_motor_efforts
fig_motor = section6_plot_motor_efforts(std_deepc.t, std_deepc.control.motors, std_mpc.control.motors);
section6_export_figure(fig_motor, fullfile(fig_dir, 'B_motor_efforts'));

% B_state_machine
fig_state = section6_plot_state_machine(std_deepc.t, std_deepc.state_machine.state_id, std_mpc.state_machine.state_id);
section6_export_figure(fig_state, fullfile(fig_dir, 'B_state_machine'));

% B_safety_margin
fig_margin = section6_plot_safety_margin(std_deepc.t, std_deepc.safety.h, std_mpc.safety.h);
section6_export_figure(fig_margin, fullfile(fig_dir, 'B_safety_margin'));

close all;
disp('Scenario B completed.');
