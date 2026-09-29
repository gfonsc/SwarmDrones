% run_regenerate_corrected.m
base_dir = 'C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch';
out_dir = fullfile(base_dir, 'validation_audit', 'figures', 'corrected');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

deepc = load(fullfile(base_dir, 'results', 'iterative_obstacle_results.mat'));
mpc = load(fullfile(base_dir, 'mpc_baseline', 'results', 'mpc_obstacle_results.mat'));

td = deepc.result.log.time;
pd = deepc.result.log.pos;
rd = deepc.result.log.ref;
smd = deepc.result.safetyMargin(:);
obs = deepc.result.scenario.obstacles;
if isfield(deepc.result.scenario, 'walls')
    walls = deepc.result.scenario.walls;
else
    walls = [];
end

tm = mpc.obstacleResults.iterativeResult.log.time(:);
pm = mpc.obstacleResults.iterativeResult.log.pos;
rm = mpc.obstacleResults.iterativeResult.log.ref;
smm = mpc.obstacleResults.iterativeResult.log.safetyMargin(:);

if size(rd, 1) == 3, rd = rd'; end
if size(pd, 1) == 3, pd = pd'; end
if size(rm, 1) == 3, rm = rm'; end
if size(pm, 1) == 3, pm = pm'; end

% Plot Style
set(groot, 'defaultFigureColor', 'w', 'defaultAxesColor', 'w', 'defaultAxesGridColor', [0.75 0.75 0.75]);
set(groot, 'defaultLineLineWidth', 1.4, 'defaultAxesFontSize', 11, 'defaultAxesFontName', 'Times New Roman');

% 1. Corrected 2D Trajectory with Walls
f1 = figure('Name', '2D Trajectory', 'Position', [100, 100, 800, 600]); hold on;
plot(rd(:,1), rd(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
plot(rd(1,1), rd(1,2), 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(rd(end,1), rd(end,2), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Finish');

for i = 1:size(walls, 1)
    w = walls(i,:);
    plot([w(1) w(3)], [w(2) w(4)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
end
plot(NaN, NaN, 'k-', 'LineWidth', 2, 'DisplayName', 'Wall');

for i = 1:size(obs, 1)
    c = obs(i,1:2); r = obs(i,5); r_safe = r + 0.25;
    th = linspace(0, 2*pi, 50);
    fill(c(1) + r*cos(th), c(2) + r*sin(th), [0.7 0.7 0.7], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(c(1) + r_safe*cos(th), c(2) + r_safe*sin(th), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
fill(NaN, NaN, [0.7 0.7 0.7], 'EdgeColor', 'none', 'DisplayName', 'Obstacle');

plot(pd(:,1), pd(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
plot(pm(:,1), pm(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
axis equal; grid on; box on; legend('Location', 'best'); xlabel('X [m]'); ylabel('Y [m]');
exportgraphics(f1, fullfile(out_dir, 'corrected_2d_trajectory_with_obstacles.png'), 'Resolution', 300);

% 2. Corrected XYZ Tracking
f2 = figure('Name', 'XYZ Tracking', 'Position', [100, 100, 800, 800]);
labels = {'X [m]', 'Y [m]', 'Z [m]'};
for i=1:3
    subplot(3,1,i); hold on;
    plot(td, rd(1:length(td),i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference (DeePC time)');
    plot(td, pd(1:length(td),i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
    plot(tm, pm(1:length(tm),i), 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
    ylabel(labels{i}); grid on; box on;
    if i==1, legend('Location', 'best'); end
    if i==3, xlabel('Time [s]'); end
end
exportgraphics(f2, fullfile(out_dir, 'corrected_xyz_tracking.png'), 'Resolution', 300);

% 3. Corrected Safety Margin
f3 = figure('Name', 'Safety Margin', 'Position', [100, 100, 800, 400]); hold on;
plot(td, smd, 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
plot(tm, smm, 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
plot([0 max(td)], [0 0], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Threshold (h=0)');
xlabel('Time [s]'); ylabel('Safety Margin h(k) [m]'); grid on; box on; legend('Location', 'best');
exportgraphics(f3, fullfile(out_dir, 'corrected_safety_margin.png'), 'Resolution', 300);

% 4. Corrected Tracking Error Norm
err_d = sqrt(sum((pd - rd(1:length(td),:)).^2, 2));
err_m = sqrt(sum((pm - rm(1:length(tm),:)).^2, 2));
f4 = figure('Name', 'Tracking Error Norm', 'Position', [100, 100, 800, 400]); hold on;
plot(td, err_d, 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
plot(tm, err_m, 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
xlabel('Time [s]'); ylabel('Error Norm ||p_{ref} - p|| [m]'); grid on; box on; legend('Location', 'best');
exportgraphics(f4, fullfile(out_dir, 'corrected_tracking_error_norm.png'), 'Resolution', 300);

close all;
