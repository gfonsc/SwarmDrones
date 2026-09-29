clear; clc; close all;

paths = run_00_setup_final_comparison_paths();
S = load(fullfile(paths.resultsDir, 'final_comparison_results.mat'), 'comparison');
comparison = S.comparison;

final_plot_2d_comparison(comparison, fullfile(paths.paperReadyDir, 'Fig_2D_DeePC_vs_MPC.png'));
final_plot_3d_comparison(comparison, fullfile(paths.paperReadyDir, 'Fig_3D_DeePC_vs_MPC.png'));
final_plot_combined_2d_3d(comparison, fullfile(paths.paperReadyDir, 'Fig_Combined_2D_3D_DeePC_vs_MPC.png'));
final_plot_xyz_comparison(comparison, fullfile(paths.paperReadyDir, 'Fig_XYZ_DeePC_vs_MPC.png'));
final_plot_motor_efforts_comparison(comparison, fullfile(paths.paperReadyDir, 'Fig_MotorEfforts_DeePC_vs_MPC.png'));

fprintf('\nFinal paper figures generated:\n');
fprintf('  %s\n', fullfile(paths.paperReadyDir, 'Fig_2D_DeePC_vs_MPC.png'));
fprintf('  %s\n', fullfile(paths.paperReadyDir, 'Fig_3D_DeePC_vs_MPC.png'));
fprintf('  %s\n', fullfile(paths.paperReadyDir, 'Fig_Combined_2D_3D_DeePC_vs_MPC.png'));
fprintf('  %s\n', fullfile(paths.paperReadyDir, 'Fig_XYZ_DeePC_vs_MPC.png'));
fprintf('  %s\n', fullfile(paths.paperReadyDir, 'Fig_MotorEfforts_DeePC_vs_MPC.png'));
