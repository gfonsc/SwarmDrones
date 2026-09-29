function metrics = final_compute_metrics(result)
%FINAL_COMPUTE_METRICS Compute common final-comparison metrics.

    pos = result.log.pos;
    ref = result.log.ref;
    n = min(size(pos, 2), size(ref, 2));
    pos = pos(:, 1:n);
    ref = ref(:, 1:n);
    err = pos - ref;
    errNorm = vecnorm(err, 2, 1);

    effort = result.motorEffort.effort(:, 1:n);
    if numel(result.log.time) >= 2
        dt = median(diff(result.log.time));
    else
        dt = 1;
    end

    h = result.safetyMargin(:);
    if numel(h) > n
        h = h(1:n);
    end

    metrics = struct();
    metrics.RMSE_x = sqrt(mean(err(1, :).^2));
    metrics.RMSE_y = sqrt(mean(err(2, :).^2));
    metrics.RMSE_z = sqrt(mean(err(3, :).^2));
    metrics.RMSE_total = sqrt(mean(errNorm.^2));
    metrics.MAE_x = mean(abs(err(1, :)));
    metrics.MAE_y = mean(abs(err(2, :)));
    metrics.MAE_z = mean(abs(err(3, :)));
    metrics.max_error_total = max(errNorm);
    metrics.final_error_total = errNorm(end);
    metrics.control_energy_motor_1 = dt * sum(effort(1, :).^2);
    metrics.control_energy_motor_2 = dt * sum(effort(2, :).^2);
    metrics.control_energy_motor_3 = dt * sum(effort(3, :).^2);
    metrics.control_energy_motor_4 = dt * sum(effort(4, :).^2);
    metrics.control_energy_total = metrics.control_energy_motor_1 + metrics.control_energy_motor_2 + ...
        metrics.control_energy_motor_3 + metrics.control_energy_motor_4;

    if size(effort, 2) >= 2
        metrics.control_variation_total = sum(diff(effort, 1, 2).^2, 'all');
    else
        metrics.control_variation_total = 0;
    end
    metrics.max_control_effort = max(abs(effort), [], 'all');

    if isempty(h) || all(isnan(h))
        metrics.min_distance_to_obstacle = NaN;
        metrics.min_h = NaN;
        metrics.num_safety_violations = NaN;
        metrics.max_violation_depth = NaN;
    else
        metrics.min_h = min(h);
        metrics.num_safety_violations = sum(h < 0);
        if any(h < 0)
            metrics.max_violation_depth = max(-h(h < 0));
        else
            metrics.max_violation_depth = 0;
        end
        if isfield(result.safetyDetail, 'obstacleClearance')
            c = result.safetyDetail.obstacleClearance(:);
            c = c(isfinite(c));
            if isempty(c)
                metrics.min_distance_to_obstacle = NaN;
            else
                metrics.min_distance_to_obstacle = min(c);
            end
        else
            metrics.min_distance_to_obstacle = NaN;
        end
    end

    metrics.mean_solver_time = mean(result.log.solveTime);
    metrics.max_solver_time = max(result.log.solveTime);
    metrics.feasibility_rate = mean(result.log.success > 0);
end
