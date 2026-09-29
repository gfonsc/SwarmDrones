function [h, details] = final_compute_safety_margin(traj, scenario, varargin)
%FINAL_COMPUTE_SAFETY_MARGIN Compute final comparison safety metrics.

    p = inputParser;
    addParameter(p, 'UseObstacles', true, @islogical);
    addParameter(p, 'UseWalls', false, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    if isempty(traj) || ~opts.UseObstacles || ~isfield(scenario, 'obstacles') || isempty(scenario.obstacles)
        n = inferTrajectoryLength(traj);
        h = nan(n, 1);
        details = struct();
        details.obstacleClearance = nan(n, 1);
        details.wallClearance = nan(n, 1);
        details.nearestSource = repmat({'none'}, n, 1);
        details.nearestIndex = zeros(n, 1);
        return;
    end

    [h, details] = branch_compute_safety_margin(traj, scenario, ...
        'UseWalls', opts.UseWalls, 'UseObstacles', opts.UseObstacles);
end

function n = inferTrajectoryLength(traj)
    if isempty(traj)
        n = 0;
    elseif size(traj, 2) == 3
        n = size(traj, 1);
    else
        n = size(traj, 2);
    end
end
