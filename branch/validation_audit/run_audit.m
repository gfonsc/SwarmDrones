% run_audit.m
base_dir = 'C:\Users\Gabriel\Desktop\EmBuscadoVooInexplicavel\SwarmDrones\branch';

deepc_files = {'baseline_results.mat', 'soft_obstacle_results.mat', 'iterative_obstacle_results.mat'};
mpc_files = {'mpc_baseline_results.mat', 'mpc_obstacle_results.mat'};

fid = fopen(fullfile(base_dir, 'validation_audit', 'tables', 'result_signal_inventory.csv'), 'w');
fprintf(fid, 'result_file,controller,scenario,time_variable,num_samples,time_start,time_end,reference_available,trajectory_available,prediction_available,state_available,input_available,motor_channels_available,euler_available,safety_margin_available,obstacle_available,notes\n');

% Process DeePC
for i=1:length(deepc_files)
    fpath = fullfile(base_dir, 'results', deepc_files{i});
    if exist(fpath, 'file')
        d = load(fpath);
        log = d.result.log;
        
        has_t = isfield(log, 'time');
        if has_t, t = log.time(:); N = length(t); t_s = t(1); t_e = t(end); else, t=[]; N=0; t_s=NaN; t_e=NaN; end
        
        has_ref = isfield(log, 'ref') && ~isempty(log.ref);
        has_traj = isfield(log, 'pos') && ~isempty(log.pos);
        has_pred = isfield(log, 'pred') && ~isempty(log.pred);
        has_state = false; % DeePC only maps inputs to outputs
        has_u = isfield(log, 'u') && ~isempty(log.u);
        if has_u
            if min(size(log.u)) == 4
                u_channels = 4;
            else
                u_channels = size(log.u, 2);
            end
        else
            u_channels = 0;
        end
        if has_u && size(log.u,1)==4 && size(log.u,2)>4; u_channels=4; elseif has_u && size(log.u,2)==4; u_channels=4; end
        
        has_euler = isfield(log, 'euler') && ~isempty(log.euler);
        has_safety = isfield(log, 'safetyMargin') && ~isempty(log.safetyMargin);
        has_obs = isfield(d.result, 'scenario') && isfield(d.result.scenario, 'obstacles');
        
        fprintf(fid, '%s,DeePC,%s,%d,%d,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n', ...
            deepc_files{i}, deepc_files{i}, has_t, N, t_s, t_e, has_ref, has_traj, has_pred, has_state, has_u, u_channels, has_euler, has_safety, has_obs);
    end
end

% Process MPC
for i=1:length(mpc_files)
    fpath = fullfile(base_dir, 'mpc_baseline', 'results', mpc_files{i});
    if exist(fpath, 'file')
        d = load(fpath);
        
        % MPC obstacle results have 'obstacleResults' containing 'softResult' and 'iterativeResult'
        if isfield(d, 'obstacleResults')
            log = d.obstacleResults.iterativeResult.log;
            scen = d.obstacleResults.iterativeResult.scenario;
        elseif isfield(d, 'baselineResult')
            log = d.baselineResult.log;
            scen = d.baselineResult.scenario;
        elseif isfield(d, 'result')
            log = d.result.log;
            scen = d.result.scenario;
        end
        
        has_t = isfield(log, 'time');
        if has_t, t = log.time(:); N = length(t); t_s = t(1); t_e = t(end); else, t=[]; N=0; t_s=NaN; t_e=NaN; end
        
        has_ref = isfield(log, 'ref') && ~isempty(log.ref);
        has_traj = isfield(log, 'pos') && ~isempty(log.pos);
        has_pred = isfield(log, 'pred') && ~isempty(log.pred);
        has_state = isfield(log, 'state') && ~isempty(log.state);
        has_u = isfield(log, 'u') && ~isempty(log.u);
        if has_u && size(log.u,1)==4 && size(log.u,2)>4; u_channels=4; elseif has_u && size(log.u,2)==4; u_channels=4; else u_channels=size(log.u,2); end
        
        has_euler = isfield(log, 'euler') && ~isempty(log.euler);
        has_safety = isfield(log, 'safetyMargin') && ~isempty(log.safetyMargin);
        has_obs = isfield(scen, 'obstacles');
        
        fprintf(fid, '%s,MPC,%s,%d,%d,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n', ...
            mpc_files{i}, mpc_files{i}, has_t, N, t_s, t_e, has_ref, has_traj, has_pred, has_state, has_u, u_channels, has_euler, has_safety, has_obs);
    end
end
fclose(fid);
