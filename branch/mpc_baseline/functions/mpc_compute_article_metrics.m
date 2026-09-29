function metrics = mpc_compute_article_metrics(data, scenario)
%MPC_COMPUTE_ARTICLE_METRICS Compute article-style tracking, safety, and effort metrics.

    if nargin < 2
        scenario = [];
    end

    pos = getMatrixField(data, {'pos', 'traj', 'trajectory'});
    ref = getMatrixField(data, {'ref', 'reference', 'referenceTrajectory'});
    u = getControlField(data, {'u', 'control', 'controlHistory'});
    time = getVectorField(data, {'time', 't'});
    solverTime = getVectorField(data, {'solveTime', 'solve_times', 'solverTime'});
    success = getVectorField(data, {'success', 'feasible', 'feasibility'});

    if isempty(time)
        dt = 1;
    elseif numel(time) >= 2
        dt = median(diff(time));
    else
        dt = 1;
    end

    [pos, ref] = alignPositionAndReference(pos, ref);
    err = pos - ref;

    metrics = struct();
    metrics.RMSE_x = computeRmse(err, 1);
    metrics.RMSE_y = computeRmse(err, 2);
    metrics.RMSE_z = computeRmse(err, 3);
    metrics.RMSE_total = computeTotalRmse(err);
    metrics.MAE_x = computeMae(err, 1);
    metrics.MAE_y = computeMae(err, 2);
    metrics.MAE_z = computeMae(err, 3);
    metrics.max_error_x = computeMaxAbs(err, 1);
    metrics.max_error_y = computeMaxAbs(err, 2);
    metrics.max_error_z = computeMaxAbs(err, 3);
    metrics.max_error_total = computeMaxNorm(err);
    metrics.final_error_total = computeFinalNorm(err);

    if isempty(u)
        metrics.control_energy_total = NaN;
        metrics.control_energy_per_motor = nan(4, 1);
        metrics.control_variation_total = NaN;
        metrics.control_variation_per_motor = nan(4, 1);
    else
        if size(u, 2) ~= size(pos, 1) && size(u, 1) == size(pos, 1)
            u = u';
        end
        if size(u, 1) ~= 4 && size(u, 2) == 4
            u = u';
        end
        metrics.control_energy_per_motor = dt * sum(u.^2, 2);
        metrics.control_energy_total = safeSum(metrics.control_energy_per_motor);
        if size(u, 2) >= 2
            du = diff(u, 1, 2);
            metrics.control_variation_per_motor = sum(du.^2, 2);
            metrics.control_variation_total = safeSum(metrics.control_variation_per_motor);
        else
            metrics.control_variation_per_motor = zeros(size(u, 1), 1);
            metrics.control_variation_total = 0;
        end
    end

    if ~isempty(scenario) && ~isempty(pos)
        [h, detail] = mpc_compute_safety_margin(pos, scenario);
        metrics.min_h = minOrNaN(h);
        metrics.num_safety_violations = sum(h < 0, 'omitnan');
        if any(h < 0)
            metrics.max_violation_depth = max(-h(h < 0));
        else
            metrics.max_violation_depth = 0;
        end

        obstacleClearance = fieldOrDefault(detail, 'obstacleClearance', nan(size(h)));
        metrics.min_distance_to_obstacle = minOrNaN(obstacleClearance);
    else
        metrics.min_distance_to_obstacle = NaN;
        metrics.min_h = NaN;
        metrics.num_safety_violations = NaN;
        metrics.max_violation_depth = NaN;
    end

    metrics.mean_solver_time = meanOrNaN(solverTime);
    metrics.max_solver_time = maxOrNaN(solverTime);

    if isempty(success)
        metrics.feasibility_rate = NaN;
    else
        metrics.feasibility_rate = mean(success > 0);
    end
end

function M = getMatrixField(s, candidates)
    M = [];
    for i = 1:numel(candidates)
        if isfield(s, candidates{i})
            M = s.(candidates{i});
            break;
        end
    end
    if isempty(M)
        return;
    end
    if size(M, 1) == 3
        M = M';
    elseif size(M, 2) ~= 3
        M = [];
    end
end

function U = getControlField(s, candidates)
    U = [];
    for i = 1:numel(candidates)
        if isfield(s, candidates{i})
            U = s.(candidates{i});
            break;
        end
    end
    if isempty(U)
        return;
    end
    if size(U, 1) == 4
        return;
    end
    if size(U, 2) == 4
        U = U';
        return;
    end
    U = [];
end

function v = getVectorField(s, candidates)
    v = [];
    for i = 1:numel(candidates)
        if isfield(s, candidates{i})
            v = s.(candidates{i});
            break;
        end
    end
    if isempty(v)
        return;
    end
    v = v(:)';
end

function [pos, ref] = alignPositionAndReference(pos, ref)
    if isempty(pos)
        pos = nan(0, 3);
    end
    if isempty(ref)
        ref = nan(size(pos, 1), 3);
    end

    n = min(size(pos, 1), size(ref, 1));
    if n == 0
        pos = nan(0, 3);
        ref = nan(0, 3);
        return;
    end
    pos = pos(1:n, :);
    ref = ref(1:n, :);
end

function value = computeRmse(err, idx)
    if isempty(err)
        value = NaN;
    else
        value = sqrt(mean(err(:, idx).^2, 'omitnan'));
    end
end

function value = computeMae(err, idx)
    if isempty(err)
        value = NaN;
    else
        value = mean(abs(err(:, idx)), 'omitnan');
    end
end

function value = computeMaxAbs(err, idx)
    if isempty(err)
        value = NaN;
    else
        value = max(abs(err(:, idx)), [], 'omitnan');
    end
end

function value = computeTotalRmse(err)
    if isempty(err)
        value = NaN;
    else
        value = sqrt(mean(sum(err.^2, 2), 'omitnan'));
    end
end

function value = computeMaxNorm(err)
    if isempty(err)
        value = NaN;
    else
        value = max(vecnorm(err, 2, 2));
    end
end

function value = computeFinalNorm(err)
    if isempty(err)
        value = NaN;
    else
        value = norm(err(end, :));
    end
end

function value = minOrNaN(x)
    if isempty(x)
        value = NaN;
    else
        value = min(x, [], 'omitnan');
    end
end

function value = meanOrNaN(x)
    if isempty(x)
        value = NaN;
    else
        value = mean(x, 'omitnan');
    end
end

function value = maxOrNaN(x)
    if isempty(x)
        value = NaN;
    else
        value = max(x, [], 'omitnan');
    end
end

function value = fieldOrDefault(s, fieldName, defaultValue)
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end

function value = safeSum(x)
    x = x(:);
    x = x(isfinite(x));
    if isempty(x)
        value = NaN;
    else
        value = sum(x);
    end
end
