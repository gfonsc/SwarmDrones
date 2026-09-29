% run_01_section6_no_obstacle.m
% Load baseline results (which ignore obstacles) and plot them without rendering obstacles.
addpath('functions');

deepc_file = fullfile('..', 'results', 'baseline_results.mat');
mpc_file = fullfile('..', 'mpc_baseline', 'results', 'mpc_baseline_results.mat');

data_deepc = load(deepc_file);
data_mpc = load(mpc_file);

% Create standardized structs
% Scenario has obstacles, but we omit them for Scenario A plots
std_deepc = section6_standardize_result_struct(data_deepc.result.log, 'DeePC', 'Scenario A - No Obstacles', data_deepc.result.scenario);
std_mpc = section6_standardize_result_struct(data_mpc.result.log, 'MPC', 'Scenario A - No Obstacles', data_deepc.result.scenario);

% Save standardized data
out_dir = fullfile('data');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
save(fullfile(out_dir, 'scenario_A_no_obstacle_results.mat'), 'std_deepc', 'std_mpc');

% Plot
fig_dir = fullfile('figures', 'scenario_A_no_obstacle');

% A_2d_trajectory
fig2d = section6_plot_2d_trajectory(std_deepc.ref.position, std_deepc.actual.position, std_mpc.actual.position, [], []);
section6_export_figure(fig2d, fullfile(fig_dir, 'A_2d_trajectory'));

% A_3d_trajectory
fig3d = section6_plot_3d_trajectory(std_deepc.ref.position, std_deepc.actual.position, std_mpc.actual.position, [], []);
section6_export_figure(fig3d, fullfile(fig_dir, 'A_3d_trajectory'));

% A_xyz_tracking
figxyz = section6_plot_xyz(std_deepc.t, std_deepc.ref.position, std_deepc.actual.position, std_mpc.actual.position);
section6_export_figure(figxyz, fullfile(fig_dir, 'A_xyz_tracking'));

% A_euler_angles
fig_euler = section6_plot_euler_angles(std_mpc.t, [], std_deepc.actual.euler, std_mpc.actual.euler);
section6_export_figure(fig_euler, fullfile(fig_dir, 'A_euler_angles'));

% A_motor_efforts
fig_motor = section6_plot_motor_efforts(std_deepc.t, std_deepc.control.motors, std_mpc.control.motors);
section6_export_figure(fig_motor, fullfile(fig_dir, 'A_motor_efforts'));

% A_state_machine
fig_state = section6_plot_state_machine(std_deepc.t, std_deepc.state_machine.state_id, std_mpc.state_machine.state_id);
section6_export_figure(fig_state, fullfile(fig_dir, 'A_state_machine'));

close all;
disp('Scenario A completed.');
