function [h, details] = mpc_compute_safety_margin(traj, scenario)
%MPC_COMPUTE_SAFETY_MARGIN Reuse the branch DeePC safety-margin geometry.

    if nargin < 2 || isempty(scenario) || isempty(traj)
        h = nan(0, 1);
        details = struct();
        return;
    end

    [h, details] = branch_compute_safety_margin(traj, scenario);
end
