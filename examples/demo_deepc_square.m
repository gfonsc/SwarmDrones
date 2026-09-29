% demo_deepc_square.m
% Validates DeePC trajectory tracking natively using extracted dataset.
% Scenario: Small square trajectory inside the benchmark data envelope.

clc; clear; close all;

%% 1. Configured Paths
base_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(base_dir, 'controllers', 'deepc'));
addpath(fullfile(base_dir, 'database'));

%% 2. Load Offline Data (from IDSIA Benchmark)
fprintf('Loading Offline Data from Database...\n');
T_ini = 6;
T_f   = 25;
[U_train, Y_train, ~] = loadBenchmarkData('random_run1.csv', T_ini, T_f, ...
    'IncludeAttitude', true);

%% 3. DeePC Hyperparameters Setup
params.T_ini = T_ini;
params.T_f = T_f;
params.lambda_y = 1e3; % Initial condition mismatch in normalized coordinates
params.lambda_g = 10;  % Regularization in normalized coordinates
params.qNorm    = 2;   % 2-norm for regularization

m = size(U_train,1);
p = size(Y_train,1);

% Define physics constants
mass = 0.045; g = 9.81; T_hover = mass * g;
u_offset = [T_hover; 0; 0; 0];

% Use a data-centered output coordinate frame. DeePC then tracks small
% deviations around the operating region actually present in random_run1.
pos_min = min(Y_train(1:3, :), [], 2);
pos_max = max(Y_train(1:3, :), [], 2);
pos_mid = median(Y_train(1:3, :), 2);
y_offset = zeros(p, 1);
y_offset(1:3) = pos_mid;

% Normalize each channel before DeePC. Without this, meters, radians,
% rad/s, newtons, and tiny torques fight inside the same QP.
u_floor = [0.01; 1e-4; 1e-4; 1e-5];
y_floor = [0.05; 0.05; 0.05; 0.10; 0.10; 0.10; ...
           0.05; 0.05; 0.05; 0.10; 0.10; 0.10];
u_scale = max(max(abs(U_train - u_offset), [], 2), u_floor);
y_scale = max(max(abs(Y_train - y_offset), [], 2), y_floor);

% Cost Matrices in normalized coordinates.
params.Q = diag([100, 100, 30, 2, 2, 2, 5, 5, 1, 1, 1, 1]);
params.R = 0.01 * eye(m);

% Constraints in physical coordinates, derived from the training envelope.
% The DeePC optimizer receives the same bounds shifted into delta-u space.
u_span = max(U_train, [], 2) - min(U_train, [], 2);
u_cmd_min = min(U_train, [], 2) - 0.10 * u_span;
u_cmd_max = max(U_train, [], 2) + 0.10 * u_span;
u_cmd_min(1) = max(0, u_cmd_min(1));
params.u_min = -inf(m,1);
params.u_max = inf(m,1);
params.y_min = -inf(p,1);
params.y_max = inf(p,1);
% params.y_min(3) = 0; % Don't go below ground

%% 4. Offline Setup Step
% Note: we center data around the local benchmark operating point to avoid
% asking DeePC to extrapolate far outside the collected trajectory.

U_train_LTI = (U_train - u_offset) ./ u_scale;
Y_train_LTI = (Y_train - y_offset) ./ y_scale;

% Offline Build
disp('Building DeePC Offline Model...');
model = deepcSetup(U_train_LTI, Y_train_LTI, params);

%% 5. Behavioral Simulation Model
% For this integration test, the closed-loop "plant" is the next output
% predicted by the DeePC behavioral model. This validates whether the
% database Hankel model can generate a trajectory that follows the reference.

%% 6. Simulation Variables
dt = 0.01; % 100Hz
t_total = 10; % Short integration test
steps = t_total / dt;

% Loggers
log_pos = zeros(3, steps);
log_ref = zeros(3, steps);
log_u   = zeros(4, steps);
log_err = zeros(1, steps);
log_success = false(1, steps);

%% 7. Reference Trajectory Generation
% Build a conservative square that stays inside the offline data envelope.
xy_span = pos_max(1:2) - pos_min(1:2);
side = min([0.25; 0.35 * xy_span(:)]);
square_origin = pos_mid + [-side/2; -side/2; 0];
WP = [square_origin, ...
      square_origin + [side; 0; 0], ...
      square_origin + [side; side; 0], ...
      square_origin + [0; side; 0], ...
      square_origin, ...
      pos_mid] ;
T_WP = linspace(0, t_total, size(WP, 2)); % Times crossing waypoints

% Initialize at the first waypoint so the demo measures tracking, not recovery
% from an artificial initial jump.
y_state = ([WP(:, 1); zeros(p - 3, 1)] - y_offset) ./ y_scale;

% Buffers for Past Trajectory
u_ini_hist = repmat(zeros(m, 1), 1, T_ini); % Normalized input
y_ini_hist = repmat(y_state, 1, T_ini);

%% 8. Main Loop
disp('Starting Software-In-Loop Simulation...');

solverOpts = struct('solver', 'auto', 'verbose', false);

for k = 1:steps
    t_curr = k * dt;
    
    % Interpolate current reference for logging
    p_ref = interp1(T_WP, WP', min(t_curr, T_WP(end)))';

    % Build the full future reference over the DeePC prediction horizon.
    % Repeating only the current point makes the optimizer reactive and
    % produces corner-cutting on moving trajectories.
    r_horizon = zeros(p, T_f);
    for j = 1:T_f
        t_ref = min(t_curr + (j - 1) * dt, T_WP(end));
        p_ref_j = interp1(T_WP, WP', t_ref)';
        r_horizon(:, j) = ([p_ref_j; zeros(p - 3, 1)] - y_offset) ./ y_scale;
    end
    
    % Step DeePC (Note: working in Delta (LTI) coordinates)
    u_ini_vec = u_ini_hist(:);
    y_ini_vec = y_ini_hist(:);
    
    % The deepcAlgorithm returns optimal Delta U
    [u_opt_LTI, y_pred, info] = deepcAlgorithm(model, u_ini_vec, y_ini_vec, r_horizon(:), solverOpts);
    
    % Convert Optimal Delta back to Physical Input
    if ~info.success || isempty(u_opt_LTI) || isempty(y_pred) || ...
            any(~isfinite(u_opt_LTI)) || any(~isfinite(y_pred))
        if isfield(info, 'solsta')
            warning('QP Failed at t=%.2f (exitflag=%d, solsta=%s). Hovering.', ...
                t_curr, info.exitflag, info.solsta);
        else
            warning('QP Failed at t=%.2f (exitflag=%d). Hovering.', ...
                t_curr, info.exitflag);
        end
        u_norm = zeros(m, 1);
        y_state = y_ini_hist(:, end);
    else
        u_norm = u_opt_LTI(1:m);
        y_state = y_pred(1:p);
    end

    u_cmd = u_norm .* u_scale + u_offset;
    u_cmd = max(u_cmd_min, min(u_cmd_max, u_cmd));
    u_norm = (u_cmd - u_offset) ./ u_scale;
    
    % Extract measured outputs in normalized coordinates
    y_meas = y_state;
    y_abs = y_state .* y_scale + y_offset;
    
    % Update histories (sliding window shift)
    u_ini_hist = [u_ini_hist(:, 2:end), u_norm];
    y_ini_hist = [y_ini_hist(:, 2:end), y_meas];
    
    % Logging
    log_pos(:, k) = y_abs(1:3);
    log_ref(:, k) = p_ref;
    log_u(:, k)   = u_cmd;
    log_err(k) = norm(y_abs(1:3) - p_ref);
    log_success(k) = info.success;
    
    if mod(k, 100) == 0
        fprintf('Time: %.1f s | Pos: [%.3f, %.3f, %.3f] | Ref: [%.3f, %.3f, %.3f]\n', ...
            t_curr, y_abs(1), y_abs(2), y_abs(3), p_ref(1), p_ref(2), p_ref(3));
    end
end
disp('Simulation Complete!');

fprintf('QP success rate: %.1f%%\n', 100 * mean(log_success));
fprintf('Position RMSE:   %.4f m\n', sqrt(mean(log_err.^2)));
fprintf('Max position error: %.4f m\n', max(log_err));

%% 9. Plot 3D Flight
figure('Name', 'DeePC 3D Square Flight Map', 'Position', [100 100 800 600]);
plot3(log_pos(1,:), log_pos(2,:), log_pos(3,:), 'b-', 'LineWidth', 2); hold on;
plot3(WP(1,:), WP(2,:), WP(3,:), 'ro-', 'MarkerSize', 8, 'LineWidth', 2);
plot3(log_pos(1,end), log_pos(2,end), log_pos(3,end), 'k*', 'MarkerSize', 10);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('DeePC Quadrotor Path Tracking');
legend('DeePC Actual Flight', 'Reference Waypoints', 'Landing Location');

all_plot_points = [log_pos, log_ref, WP];
plot_min = min(all_plot_points, [], 2);
plot_max = max(all_plot_points, [], 2);
plot_center = (plot_min + plot_max) / 2;
plot_span = max(plot_max - plot_min);
plot_span = max(plot_span, side);
plot_pad = 0.15 * plot_span;
plot_half = plot_span / 2 + plot_pad;

xlim(plot_center(1) + [-plot_half, plot_half]);
ylim(plot_center(2) + [-plot_half, plot_half]);
zlim(plot_center(3) + [-plot_half, plot_half]);
axis vis3d;
daspect([1 1 1]);
view(45, 30);

figure('Name', 'DeePC XY Tracking', 'Position', [950 100 650 600]);
plot(log_ref(1,:), log_ref(2,:), 'r--', 'LineWidth', 2); hold on;
plot(log_pos(1,:), log_pos(2,:), 'b-', 'LineWidth', 2);
plot(WP(1,:), WP(2,:), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot(log_pos(1,end), log_pos(2,end), 'k*', 'MarkerSize', 10);
grid on;
axis equal;
xlim(plot_center(1) + [-plot_half, plot_half]);
ylim(plot_center(2) + [-plot_half, plot_half]);
xlabel('X (m)'); ylabel('Y (m)');
title('DeePC XY Tracking');
legend('Reference', 'DeePC Actual', 'Waypoints', 'Final Position', 'Location', 'best');
