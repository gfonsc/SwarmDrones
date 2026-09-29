% demo_load_benchmark.m
% Simple script to verify the new benchmark data loader

clc; clear; close all;

% DeePC Parameters
T_ini = 6;
T_f = 30;

% Load data
[U, Y, t] = loadBenchmarkData('random_run1.csv', T_ini, T_f);

% Visualize Output Dimensions
fprintf('\n--- Dataset Extracted ---\n');
fprintf('U Size: [%d x %d] (T, tau_x, tau_y, tau_z)\n', size(U, 1), size(U, 2));
fprintf('Y Size: [%d x %d] (pos, vel, rates)\n', size(Y, 1), size(Y, 2));

% Plot Results to verify filtering and dynamics
figure('Name', 'Benchmark Data Processing', 'Position', [100, 100, 1000, 600]);

subplot(3,1,1);
plot(t, Y(1:3, :));
title('Position (Filtered 10Hz)');
ylabel('m');
legend('x','y','z');
grid on;

subplot(3,1,2);
plot(t, Y(7:9, :));
title('Angular Rates (Filtered 18Hz)');
ylabel('rad/s');
legend('wx','wy','wz');
grid on;

subplot(3,1,3);
plot(t, U(1, :));
title('Thrust (Filtered 20Hz)');
ylabel('N');
xlabel('Time (s)');
grid on;
