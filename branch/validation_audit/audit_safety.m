% audit_safety.m
base_dir = 'C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch';

deepc_file = fullfile(base_dir, 'results', 'iterative_obstacle_results.mat');
mpc_file = fullfile(base_dir, 'mpc_baseline', 'results', 'mpc_obstacle_results.mat');

fid = fopen(fullfile(base_dir, 'validation_audit', 'tables', 'safety_metrics_audit.csv'), 'w');
fprintf(fid, 'controller,scenario,method,min_distance_to_obstacle,min_h,num_safety_violations,max_violation_depth,worst_violation_time,closest_obstacle_id,safety_respected,constraint_inside_optimization,notes\n');

d1 = load(deepc_file);
log_d = d1.result.log;
sm_d = d1.result.safetyMargin;

min_h_d = min(sm_d.margin);
num_viol_d = sum(sm_d.margin < 0);
max_viol_d = abs(min(min_h_d, 0));
[~, worst_idx_d] = min(sm_d.margin);
worst_time_d = log_d.time(worst_idx_d);
% To get closest obstacle, we can find it by distance
obs_clearances_d = pdist2(log_d.pos(worst_idx_d, 1:2), d1.result.scenario.obstacles(:, 1:2)) - d1.result.scenario.obstacles(:, 5)';
[min_dist_d, closest_obs_d] = min(obs_clearances_d);

fprintf(fid, 'DeePC,Scenario B,Iterative,%f,%f,%d,%f,%f,%d,No,Yes,Soft constraints allowed minor penetration\n', ...
    min_dist_d, min_h_d, num_viol_d, max_viol_d, worst_time_d, closest_obs_d);

d2 = load(mpc_file);
log_m = d2.obstacleResults.iterativeResult.log;
sm_m = log_m.safetyMargin;

min_h_m = min(sm_m.margin);
num_viol_m = sum(sm_m.margin < 0);
max_viol_m = abs(min(min_h_m, 0));
[~, worst_idx_m] = min(sm_m.margin);
worst_time_m = log_m.time(worst_idx_m);
obs_clearances_m = pdist2(log_m.pos(worst_idx_m, 1:2), d1.result.scenario.obstacles(:, 1:2)) - d1.result.scenario.obstacles(:, 5)';
[min_dist_m, closest_obs_m] = min(obs_clearances_m);

fprintf(fid, 'MPC,Scenario B,Iterative,%f,%f,%d,%f,%f,%d,No,Yes,Soft constraints allowed minor penetration\n', ...
    min_dist_m, min_h_m, num_viol_m, max_viol_m, worst_time_m, closest_obs_m);

fclose(fid);
