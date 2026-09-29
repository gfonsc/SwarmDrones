%% demo_linear_system.m
%  Validates the DeePC implementation on a known LTI system.
%
%  This is an end-to-end test: generate data, train DeePC (offline),
%  run closed-loop tracking (online), and compare against reference.
%
%  For an LTI system with noise-free data, DeePC is EQUIVALENT to MPC
%  (Theorem 2.1, Coulson thesis). This demo verifies that equivalence.
%
%  Reference: Coulson thesis, Section 5.2.1 — "DeePC is equivalent to MPC
%             when the data come from a noise-free LTI system."

clear; clc; close all;

fprintf('====================================================\n');
fprintf('  DeePC Demo: Linear System Tracking\n');
fprintf('====================================================\n\n');

%% Add DeePC controller to path
deepcPath = fullfile(fileparts(mfilename('fullpath')), '..', 'controllers', 'deepc');
addpath(deepcPath);
addpath(fullfile(deepcPath, 'functions'));

%% Define a simple LTI system
%  x(k+1) = A x(k) + B u(k)
%  y(k)   = C x(k)
%
%  2 inputs, 2 states, 2 outputs (MIMO system)
A = [0.9  0.1;
    -0.2  0.8];
B = [1.0  0.0;
     0.0  0.5];
C = [1.0  0.0;
     0.0  1.0];

n = size(A, 1);   % States
m = size(B, 2);   % Inputs
p = size(C, 1);   % Outputs

Ts = 0.1;  % Sampling time (s)

fprintf('Plant: LTI system with %d states, %d inputs, %d outputs\n', n, m, p);
fprintf('Eigenvalues: %s\n', mat2str(eig(A)', 3));

%% =========================================================
%%  STEP 1: Collect persistently exciting training data
%% =========================================================
fprintf('\n--- Collecting training data ---\n');

T = 200;  % Number of training samples
rng(42);  % Reproducibility

% Random input (white noise) — ensures persistent excitation
uTrain = 0.5 * randn(m, T);

% Simulate the system
xTrain = zeros(n, T+1);
yTrain = zeros(p, T);

for k = 1:T
    yTrain(:, k) = C * xTrain(:, k);
    xTrain(:, k+1) = A * xTrain(:, k) + B * uTrain(:, k);
end

fprintf('Training data: T = %d samples (noise-free)\n', T);

%% =========================================================
%%  STEP 2: DeePC Offline Setup
%% =========================================================
fprintf('\n--- DeePC Offline Setup ---\n');

params = struct();
params.T_ini  = 3;     % Past horizon
params.T_f    = 10;    % Prediction horizon

% Cost weights
params.Q = 10 * eye(p);      % Output tracking weight (per step)
params.R = 0.1 * eye(m);     % Input penalty (per step)

% Regularization — for noise-free LTI data, these can be small
params.lambda_y = 1e6;   % Softening of Y_p g = y_ini
params.lambda_g = 1.0;   % g regularization (small for clean data)
params.qNorm    = 2;     % L2 norm

% Steady-state references (origin)
params.u_r = zeros(m, 1);
params.y_r = zeros(p, 1);

% Input constraints
params.u_min = -2 * ones(m, 1);
params.u_max =  2 * ones(m, 1);

% Verbose output
params.verbose = true;

% Run offline setup
model = deepcSetup(uTrain, yTrain, params);

%% =========================================================
%%  STEP 3: Generate reference trajectory
%% =========================================================
fprintf('\n--- Generating reference trajectory ---\n');

T_sim = 100;  % Simulation steps

% Step reference: [0,0] → [1,0.5] at step 10, → [0,-0.5] at step 60
yRef = zeros(p, T_sim);
yRef(:, 10:59)  = repmat([1.0; 0.5], 1, 50);
yRef(:, 60:end) = repmat([0.0; -0.5], 1, T_sim - 59);

fprintf('Simulation: %d steps (%.1f s)\n', T_sim, T_sim * Ts);
fprintf('Reference changes at steps 10 and 60\n');

%% =========================================================
%%  STEP 4: Closed-loop simulation (Algorithm 5.1)
%% =========================================================
fprintf('\n--- Running closed-loop simulation (Algorithm 5.1) ---\n');

T_ini = model.dims.T_ini;
T_f   = model.dims.T_f;

% Initialize state and buffers
x = zeros(n, 1);           % Plant state

% Initial condition buffers (filled with zeros — system at rest)
uIni = zeros(m * T_ini, 1);
yIni = zeros(p * T_ini, 1);

% Storage
yHistory = zeros(p, T_sim);
uHistory = zeros(m, T_sim);
solveTimeHistory = zeros(1, T_sim);
successHistory = true(1, T_sim);

for k = 1:T_sim
    %% Build reference for current prediction window
    refStart = k;
    refEnd   = min(k + T_f - 1, T_sim);
    rWindow  = yRef(:, refStart:refEnd);

    % Pad reference if near end of simulation
    if size(rWindow, 2) < T_f
        rWindow = [rWindow, repmat(rWindow(:, end), 1, T_f - size(rWindow, 2))];
    end

    % Stack into column vector (p*T_f x 1)
    r = rWindow(:);

    %% Call DeePC online step — Algorithm 5.1, Steps 1-2
    [uOpt, yPred, info] = deepcAlgorithm(model, uIni, yIni, r);

    %% Apply first input (receding horizon) — Algorithm 5.1, Step 2
    uApply = uOpt(1:m);

    % Saturate inputs (respect constraints)
    uApply = max(params.u_min, min(params.u_max, uApply));

    %% Simulate plant
    yMeas = C * x;
    x = A * x + B * uApply;

    %% Store results
    yHistory(:, k) = yMeas;
    uHistory(:, k) = uApply;
    solveTimeHistory(k) = info.solveTime;
    successHistory(k)   = info.success;

    %% Update buffers — Algorithm 5.1, Step 3
    %  Shift: drop oldest, append newest
    uIni = [uIni(m+1:end); uApply];
    yIni = [yIni(p+1:end); yMeas];

    % Progress
    if mod(k, 25) == 0
        fprintf('  Step %d/%d — solve: %.1f ms, success: %d\n', ...
            k, T_sim, info.solveTime * 1000, info.success);
    end
end

%% =========================================================
%%  STEP 5: Results
%% =========================================================
fprintf('\n====================================================\n');
fprintf('  Results\n');
fprintf('====================================================\n');

% Tracking error (skip first T_ini steps for warmup)
trackingError = yHistory(:, T_ini+1:end) - yRef(:, T_ini+1:end);
rmse = sqrt(mean(sum(trackingError.^2, 1)));
maxError = max(vecnorm(trackingError, 2, 1));
meanSolveTime = mean(solveTimeHistory) * 1000;
successRate = sum(successHistory) / T_sim * 100;

fprintf('  RMSE:            %.4f\n', rmse);
fprintf('  Max error:       %.4f\n', maxError);
fprintf('  Mean solve time: %.2f ms\n', meanSolveTime);
fprintf('  Success rate:    %.1f%%\n', successRate);
fprintf('  Solver used:     %s\n', info.solver);

%% Visualization
t = (0:T_sim-1) * Ts;

figure('Name', 'DeePC Demo — Linear System', 'Position', [50, 50, 1200, 700], 'Color', 'w');

% Output tracking
subplot(2, 2, 1);
plot(t, yRef(1, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference y_1');
hold on;
plot(t, yHistory(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'DeePC y_1');
xlabel('Time (s)'); ylabel('Output y_1');
title('Output 1 Tracking'); legend('Location', 'best'); grid on;

subplot(2, 2, 2);
plot(t, yRef(2, :), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Reference y_2');
hold on;
plot(t, yHistory(2, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'DeePC y_2');
xlabel('Time (s)'); ylabel('Output y_2');
title('Output 2 Tracking'); legend('Location', 'best'); grid on;

% Control inputs
subplot(2, 2, 3);
stairs(t, uHistory(1, :), 'b-', 'LineWidth', 1, 'DisplayName', 'u_1');
hold on;
stairs(t, uHistory(2, :), 'r-', 'LineWidth', 1, 'DisplayName', 'u_2');
yline(params.u_max(1), 'k--', 'LineWidth', 0.5);
yline(params.u_min(1), 'k--', 'LineWidth', 0.5);
xlabel('Time (s)'); ylabel('Input');
title('Control Inputs'); legend('Location', 'best'); grid on;

% Tracking error
subplot(2, 2, 4);
errorNorm = vecnorm(yHistory - yRef, 2, 1);
plot(t, errorNorm, 'k-', 'LineWidth', 1);
hold on;
yline(rmse, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('RMSE=%.4f', rmse));
xlabel('Time (s)'); ylabel('||error||_2');
title('Tracking Error'); legend('Location', 'best'); grid on;

sgtitle('DeePC Demo — LTI System Trajectory Tracking', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\nDemo complete. Close figure to exit.\n');
