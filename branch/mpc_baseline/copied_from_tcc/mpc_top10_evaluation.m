%% mpc_top10_evaluation.m
% Run full simulations with top 10% best parameters from sweep
%
% Loads sweep results, selects top performers, and runs detailed tests

clear; clc; close all;

fprintf('========================================\n');
fprintf('MPC Top 10%% Parameter Evaluation\n');
fprintf('========================================\n\n');

%% Load sweep results
sweep_file = 'mpc_parameter_sweep_results.mat';
if ~exist(sweep_file, 'file')
    error('Run mpc_parameter_sweep.m first');
end

load(sweep_file, 'results');

fprintf('Loaded sweep results:\n');
fprintf('  Q_pos values: %s\n', mat2str(results.Q_pos_values));
fprintf('  Q_vel values: %s\n', mat2str(results.Q_vel_values));
fprintf('  R values: %s\n', mat2str(results.R_values));

%% Find top 10% configurations
mean_errors = results.mean_error(:);  % Flatten to 1D
n_total = length(mean_errors);
n_top = max(1, floor(n_total * 0.10));  % Top 10%

[sorted_errors, sorted_idx] = sort(mean_errors);
top_indices = sorted_idx(1:n_top);

fprintf('\nTop %d configurations (10%% of %d total):\n', n_top, n_total);
fprintf('%-5s | %8s | %8s | %8s | %12s\n', 'Rank', 'Q_pos', 'Q_vel', 'R', 'Mean Err (mm)');
fprintf('%s\n', repmat('-', 1, 50));

% Convert linear indices to subscripts
[i_Qpos, i_Qvel, i_R] = ind2sub(size(results.mean_error), top_indices);

top_configs = struct();
for k = 1:n_top
    top_configs(k).Q_pos = results.Q_pos_values(i_Qpos(k));
    top_configs(k).Q_vel = results.Q_vel_values(i_Qvel(k));
    top_configs(k).R = results.R_values(i_R(k));
    top_configs(k).sweep_error = sorted_errors(k);

    fprintf('%-5d | %8d | %8d | %8.3f | %12.2f\n', ...
        k, top_configs(k).Q_pos, top_configs(k).Q_vel, ...
        top_configs(k).R, top_configs(k).sweep_error);
end

%% Setup model
fprintf('\nSetting up quadrotor model...\n');
phys_params = crazyflie_params();
[A_c, B_c, C, D] = linearize_quadrotor(phys_params);

Ts = phys_params.dt;
sys_d = c2d(ss(A_c, B_c, C, D), Ts, 'zoh');
A = sys_d.A;
B = sys_d.B;

%% Generate FULL test trajectory (longer than sweep)
fprintf('Generating full test trajectory (20s)...\n');
T_sim = 20;
t = 0:Ts:T_sim;
n_steps = length(t);

% IDSIA-style square at multiple altitudes
side = 1.0;
altitudes = [0.3, 0.6, 1.0, 1.5, 1.8];
time_per_level = T_sim / length(altitudes);

x_ref = zeros(1, n_steps);
y_ref = zeros(1, n_steps);
z_ref = zeros(1, n_steps);

for k = 1:n_steps
    tk = t(k);
    level_idx = min(floor(tk / time_per_level) + 1, length(altitudes));
    current_alt = altitudes(level_idx);
    level_start = (level_idx - 1) * time_per_level;
    phase_t = mod(tk - level_start, time_per_level) / time_per_level;

    if phase_t < 0.25
        x_ref(k) = (phase_t / 0.25) * side;
        y_ref(k) = 0;
    elseif phase_t < 0.5
        x_ref(k) = side;
        y_ref(k) = ((phase_t - 0.25) / 0.25) * side;
    elseif phase_t < 0.75
        x_ref(k) = side - ((phase_t - 0.5) / 0.25) * side;
        y_ref(k) = side;
    else
        x_ref(k) = 0;
        y_ref(k) = side - ((phase_t - 0.75) / 0.25) * side;
    end

    if tk < level_start + 0.3 && level_idx > 1
        prev_alt = altitudes(level_idx - 1);
        z_ref(k) = prev_alt + ((tk - level_start) / 0.3) * (current_alt - prev_alt);
    else
        z_ref(k) = current_alt;
    end
end

vx_ref = gradient(x_ref, Ts);
vy_ref = gradient(y_ref, Ts);
vz_ref = gradient(z_ref, Ts);

ref.pos = [x_ref; y_ref; z_ref];
ref.vel = [vx_ref; vy_ref; vz_ref];

%% Fixed parameters
N_horizon = 30;
Q_ang = eye(3);
R_delta = 0.1 * eye(4);
P_term = 10;

dT_max = phys_params.T_max - phys_params.m * phys_params.g;
tau_max = max(phys_params.max_torque);
u_min = [-phys_params.m * phys_params.g * 0.5; -tau_max; -tau_max; -tau_max];
u_max = [dT_max; tau_max; tau_max; tau_max];

%% Run full simulations
fprintf('\nRunning full simulations for top %d configurations...\n', n_top);

full_results = struct();
colors = {'b', 'r', 'g', 'm', 'c', 'k', [0.5 0 0.5], [0 0.5 0.5], [0.5 0.5 0], [0.8 0.4 0]};

for cfg = 1:n_top
    fprintf('\n--- Config %d/%d: Q_pos=%d, Q_vel=%d, R=%.3f ---\n', ...
        cfg, n_top, top_configs(cfg).Q_pos, top_configs(cfg).Q_vel, top_configs(cfg).R);

    mpc_params = struct();
    mpc_params.N = N_horizon;
    mpc_params.Q_pos = top_configs(cfg).Q_pos * eye(3);
    mpc_params.Q_vel = top_configs(cfg).Q_vel * eye(3);
    mpc_params.Q_ang = Q_ang;
    mpc_params.R = top_configs(cfg).R * eye(4);
    mpc_params.R_delta = R_delta;
    mpc_params.P_term = P_term;
    mpc_params.u_min = u_min;
    mpc_params.u_max = u_max;
    mpc_params.u_prev = zeros(4, 1);

    n = size(A, 1);
    m = size(B, 2);

    x_mpc = zeros(n, n_steps);
    x_mpc(1:3, 1) = ref.pos(:, 1);
    u_mpc = zeros(m, n_steps);
    solve_times = zeros(1, n_steps);

    tic;
    for k = 1:n_steps-1
        ref_start = k;
        ref_end = min(k + mpc_params.N - 1, n_steps);

        ref_horizon.pos = ref.pos(:, ref_start:ref_end);
        ref_horizon.vel = ref.vel(:, ref_start:ref_end);

        if size(ref_horizon.pos, 2) < mpc_params.N
            n_pad = mpc_params.N - size(ref_horizon.pos, 2);
            ref_horizon.pos = [ref_horizon.pos, repmat(ref_horizon.pos(:,end), 1, n_pad)];
            ref_horizon.vel = [ref_horizon.vel, zeros(3, n_pad)];
        end

        if k > 1
            mpc_params.u_prev = u_mpc(:, k-1);
        end

        t_solve = tic;
        [u_opt, ~, info] = mpc_controller_mosek_v2(A, B, C, x_mpc(:,k), ref_horizon, mpc_params);
        solve_times(k) = toc(t_solve);

        if info.success
            u_mpc(:, k) = u_opt(:, 1);
        end

        x_mpc(:, k+1) = A * x_mpc(:, k) + B * u_mpc(:, k);

        if mod(k, 500) == 0
            fprintf('  Step %d/%d\n', k, n_steps);
        end
    end
    sim_time = toc;

    % Compute metrics
    pos_mpc = x_mpc(1:3, :);
    tracking_error = vecnorm(ref.pos - pos_mpc, 2, 1) * 1000;

    full_results(cfg).config = top_configs(cfg);
    full_results(cfg).x_mpc = x_mpc;
    full_results(cfg).u_mpc = u_mpc;
    full_results(cfg).mean_error = mean(tracking_error);
    full_results(cfg).max_error = max(tracking_error);
    full_results(cfg).rms_error = rms(tracking_error);
    full_results(cfg).solve_time = mean(solve_times) * 1000;
    full_results(cfg).sim_time = sim_time;
    full_results(cfg).tracking_error = tracking_error;

    fprintf('  Mean: %.2f mm, Max: %.2f mm, Solve: %.2f ms\n', ...
        full_results(cfg).mean_error, full_results(cfg).max_error, full_results(cfg).solve_time);
end

%% Summary
fprintf('\n========================================\n');
fprintf('FULL SIMULATION RESULTS (20s trajectory)\n');
fprintf('========================================\n\n');

fprintf('%-5s | %6s | %6s | %6s | %10s | %10s | %10s\n', ...
    'Rank', 'Q_pos', 'Q_vel', 'R', 'Mean(mm)', 'Max(mm)', 'Solve(ms)');
fprintf('%s\n', repmat('-', 1, 70));

for cfg = 1:n_top
    fprintf('%-5d | %6d | %6d | %6.3f | %10.2f | %10.2f | %10.2f\n', ...
        cfg, full_results(cfg).config.Q_pos, full_results(cfg).config.Q_vel, ...
        full_results(cfg).config.R, full_results(cfg).mean_error, ...
        full_results(cfg).max_error, full_results(cfg).solve_time);
end

%% Find best from full simulation
[~, best_idx] = min([full_results.mean_error]);
fprintf('\n*** BEST CONFIGURATION (from full test): ***\n');
fprintf('  Q_pos = %d\n', full_results(best_idx).config.Q_pos);
fprintf('  Q_vel = %d\n', full_results(best_idx).config.Q_vel);
fprintf('  R     = %.3f\n', full_results(best_idx).config.R);
fprintf('  Mean error: %.2f mm\n', full_results(best_idx).mean_error);
fprintf('  Max error:  %.2f mm\n', full_results(best_idx).max_error);

%% Plot comparison
fig = figure('Name', 'Top 10% Comparison', 'Position', [50, 50, 1400, 800], 'Color', 'w');

% 3D trajectories
subplot(2, 2, 1);
plot3(ref.pos(1,:), ref.pos(2,:), ref.pos(3,:), 'k-', 'LineWidth', 3, 'DisplayName', 'Reference');
hold on;
for cfg = 1:min(5, n_top)  % Plot top 5 only for clarity
    pos = full_results(cfg).x_mpc(1:3, :);
    plot3(pos(1,:), pos(2,:), pos(3,:), '-', 'LineWidth', 1.5, 'Color', colors{cfg}, ...
        'DisplayName', sprintf('#%d (%.1fmm)', cfg, full_results(cfg).mean_error));
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectories (Top 5)', 'FontSize', 14);
legend('Location', 'best');
grid on; view(45, 25);

% Error over time
subplot(2, 2, 2);
for cfg = 1:min(5, n_top)
    plot(t, full_results(cfg).tracking_error, '-', 'LineWidth', 1, 'Color', colors{cfg});
    hold on;
end
xlabel('Time (s)');
ylabel('Error (mm)');
title('Tracking Error Over Time', 'FontSize', 14);
grid on;

% Bar chart of mean errors
subplot(2, 2, 3);
mean_errs = [full_results.mean_error];
bar(mean_errs);
xlabel('Configuration Rank');
ylabel('Mean Error (mm)');
title('Mean Error by Configuration', 'FontSize', 14);
grid on;

% Parameter scatter
subplot(2, 2, 4);
Q_pos_all = [full_results.config];
Q_pos_vals = [Q_pos_all.Q_pos];
R_vals = [Q_pos_all.R];
scatter(Q_pos_vals, R_vals, 100, mean_errs, 'filled');
colorbar;
xlabel('Q_{pos}');
ylabel('R');
set(gca, 'YScale', 'log');
title('Error vs Parameters', 'FontSize', 14);
grid on;

sgtitle('MPC Top 10% Configuration Comparison', 'FontSize', 16, 'FontWeight', 'bold');

saveas(fig, 'mpc_top10_comparison.png');
fprintf('\nSaved: mpc_top10_comparison.png\n');

%% Save best configuration for future use
best_config = full_results(best_idx);
save('mpc_best_config.mat', 'best_config', 'full_results', 't', 'ref');
fprintf('Saved: mpc_best_config.mat\n');

fprintf('\n=== Evaluation Complete ===\n');
