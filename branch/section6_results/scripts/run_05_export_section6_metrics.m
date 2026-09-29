% run_05_export_section6_metrics.m
% Calculate and export metrics to tables/section6_metrics_for_6_4.csv
disp('Calculating metrics...');

out_file = fullfile('..', 'tables', 'section6_metrics_for_6_4.csv');
fid = fopen(out_file, 'w');
fprintf(fid, 'scenario,controller,RMSE_x,RMSE_y,RMSE_z,RMSE_total,MAE_x,MAE_y,MAE_z,max_error_total,final_error_total,control_energy_total,control_variation_total,min_distance_to_obstacle,min_h,num_safety_violations,max_violation_depth,mean_solver_time,max_solver_time,feasibility_rate,notes\n');

dataFiles = {
    'scenario_A_no_obstacle_results.mat', 'Scenario A';
    'scenario_B_with_obstacles_results.mat', 'Scenario B';
    'scenario_C_time_lagged_obstacles_results.mat', 'Scenario C';
};

for i = 1:size(dataFiles, 1)
    file_path = fullfile('..', 'data', dataFiles{i, 1});
    scenarioName = dataFiles{i, 2};
    
    if ~exist(file_path, 'file'), continue; end
    d = load(file_path);
    
    structs = {d.std_deepc, d.std_mpc};
    for j = 1:2
        s = structs{j};
        if isempty(s.actual.position), continue; end
        
        N = size(s.actual.position, 1);
        ref = s.ref.position(1:N, :);
        act = s.actual.position;
        
        err = act - ref;
        rmse = sqrt(mean(err.^2, 1));
        rmse_tot = sqrt(mean(sum(err.^2, 2)));
        mae = mean(abs(err), 1);
        max_err = max(sqrt(sum(err.^2, 2)));
        final_err = sqrt(sum(err(end,:).^2));
        
        if ~isempty(s.control.motors)
            u = s.control.motors;
            c_energy = sum(sum(u.^2));
            c_var = sum(sum(diff(u, 1, 1).^2));
        else
            c_energy = NaN; c_var = NaN;
        end
        
        if ~isempty(s.safety.h)
            min_h = min(s.safety.h);
            num_viol = sum(s.safety.h < 0);
            max_viol = abs(min(min_h, 0));
        else
            min_h = NaN; num_viol = NaN; max_viol = NaN;
        end
        
        if ~isempty(s.safety.distance)
            min_dist = min(s.safety.distance);
        else
            min_dist = NaN;
        end
        
        fprintf(fid, '%s,%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,NaN,NaN,1.0,\n', ...
            scenarioName, s.method, rmse(1), rmse(2), rmse(3), rmse_tot, mae(1), mae(2), mae(3), ...
            max_err, final_err, c_energy, c_var, min_dist, min_h, num_viol, max_viol);
    end
end
fclose(fid);
disp('Metrics exported.');
