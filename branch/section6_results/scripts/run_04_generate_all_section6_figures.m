% run_04_generate_all_section6_figures.m
% Execute all scenario scripts and generate paper-ready figures
disp('Starting Section 6 data extraction and figure generation...');

% Run Scenario A
run_01_section6_no_obstacle;

% Run Scenario B
run_02_section6_with_obstacles;

% Run Scenario C
run_03_section6_time_lagged_obstacles;

% Generate paper-ready copies
paper_ready_dir = fullfile('figures', 'paper_ready');
if ~exist(paper_ready_dir, 'dir'), mkdir(paper_ready_dir); end

disp('Copying files to paper_ready...');

copyfile(fullfile('figures', 'scenario_A_no_obstacle', 'A_2d_trajectory.png'), fullfile(paper_ready_dir, 'Fig6a_no_obstacle_2d_trajectory.png'));
copyfile(fullfile('figures', 'scenario_B_with_obstacles', 'B_2d_trajectory_obstacles.png'), fullfile(paper_ready_dir, 'Fig6b_obstacle_2d_trajectory.png'));
copyfile(fullfile('figures', 'scenario_B_with_obstacles', 'B_3d_trajectory_obstacles.png'), fullfile(paper_ready_dir, 'Fig6c_obstacle_3d_trajectory.png'));
copyfile(fullfile('figures', 'scenario_B_with_obstacles', 'B_xyz_tracking.png'), fullfile(paper_ready_dir, 'Fig6d_xyz_tracking_comparison.png'));
copyfile(fullfile('figures', 'scenario_B_with_obstacles', 'B_euler_angles.png'), fullfile(paper_ready_dir, 'Fig6e_euler_angles_comparison.png'));
copyfile(fullfile('figures', 'scenario_B_with_obstacles', 'B_motor_efforts.png'), fullfile(paper_ready_dir, 'Fig6f_motor_efforts_comparison.png'));
copyfile(fullfile('figures', 'scenario_B_with_obstacles', 'B_state_machine.png'), fullfile(paper_ready_dir, 'Fig6g_state_machine_activation.png'));
copyfile(fullfile('figures', 'scenario_B_with_obstacles', 'B_safety_margin.png'), fullfile(paper_ready_dir, 'Fig6h_safety_margin_comparison.png'));

disp('Paper-ready figures generated successfully.');
