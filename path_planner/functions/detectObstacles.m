function [detected, minDist, obsAngle, canUp, canDown] = ...
    detectObstacles(pos, yaw, obstacles, params, constants)
% DETECTOBSTACLES  Detect obstacles within the drone's field of view.
%
%   [detected, minDist, obsAngle, canUp, canDown] = ...
%       detectObstacles(pos, yaw, obstacles, params, constants)
%
%   Implements obstacle detection using one of three FOV modes:
%     'cone'     — Traditional cone-shaped FOV (Nycolas original)
%     'corridor' — Rectangular corridor in local frame (TCC implementation)
%     'dual'     — Dual vector-pair positive span (Nycolas §7.1, Fig. 23)
%
%   For each detected obstacle, determines whether vertical escape is
%   possible (can_go_up / can_go_down) by checking if the altitude ± step
%   clears the obstacle's vertical extent.
%
%   Inputs:
%       pos       - Drone position [x; y; z] (3x1)
%       yaw       - Drone yaw angle [rad]
%       obstacles - Obstacle matrix (N x 5): [x, y, z_min, z_max, radius]
%       params    - Parameter struct from pathPlannerSetup
%       constants - Constants struct from pathPlannerSetup
%
%   Outputs:
%       detected  - true if at least one obstacle is in FOV and within
%                   safe_distance at current altitude
%       minDist   - Distance to nearest detected obstacle [m] (inf if none)
%       obsAngle  - Angle to nearest obstacle relative to heading [rad]
%       canUp     - true if ascending clears the blocking obstacle
%       canDown   - true if descending clears the blocking obstacle
%
%   Reference:
%     - Nycolas Ramires, TCC — Eqs. 1–5 (FOV detection)
%     - Nycolas §7.1, Fig. 23 (corridor FOV)

    detected = false;
    minDist  = inf;
    obsAngle = 0;
    canUp    = false;
    canDown  = false;

    if isempty(obstacles)
        return;
    end

    nObs = size(obstacles, 1);
    blocking_z_max = 0;
    blocking_z_min = inf;

    fov_half = constants.fov_half;

    for i = 1:nObs
        obs_xy  = obstacles(i, 1:2)';
        z_min   = obstacles(i, 3);
        z_max   = obstacles(i, 4);
        radius  = obstacles(i, 5);

        %% Distance in XY plane
        to_obs   = obs_xy - pos(1:2);
        dist_xy  = norm(to_obs) - radius;

        %% FOV check (depends on mode)
        in_fov = false;

        % Angle to obstacle in drone-relative frame
        obs_global_angle = atan2(to_obs(2), to_obs(1)) - yaw;
        obs_global_angle = atan2(sin(obs_global_angle), cos(obs_global_angle));

        switch params.fov_mode
            case 'cone'
                % Traditional cone FOV (Nycolas original: Eqs. 1–5)
                in_fov = abs(obs_global_angle) < fov_half && ...
                         dist_xy < params.detection_range;

            case 'corridor'
                % Cone check
                in_cone = abs(obs_global_angle) < fov_half && ...
                          dist_xy < params.detection_range;

                % Corridor check (rotation to local frame)
                R = [cos(-yaw), -sin(-yaw); sin(-yaw), cos(-yaw)];
                obs_local = R * to_obs;
                in_corridor = obs_local(1) > 0 && ...
                    obs_local(1) < (params.corridor_length + radius) && ...
                    abs(obs_local(2)) < (params.corridor_width + radius);

                in_fov = in_cone || in_corridor;

            case 'dual'
                % Dual vector-pair positive span (Nycolas §7.1, Fig. 23)
                %
                % The corridor is defined by TWO pairs of boundary vectors.
                % A point must be in the positive span of BOTH pairs.
                %
                % Pair 1: Standard FOV vectors (v1, v2) rotated by yaw
                % Pair 2: Translated vectors defining the corridor walls
                %
                % Optimization: since Pair 2 is a mirror-translate of Pair 1,
                % we check using a single set of operations.

                % Cone check (Pair 1)
                in_cone = abs(obs_global_angle) < fov_half && ...
                          dist_xy < params.detection_range;

                % Corridor walls (Pair 2) — equivalent to checking that the
                % obstacle's local-Y coordinate falls within ±corridor_width
                R = [cos(-yaw), -sin(-yaw); sin(-yaw), cos(-yaw)];
                obs_local = R * to_obs;

                in_dual = in_cone && ...
                    abs(obs_local(2)) < (params.corridor_width + radius);

                % Also add close-range check (within corridor_length, any Y)
                in_close = obs_local(1) > 0 && ...
                    obs_local(1) < (params.corridor_length + radius) && ...
                    abs(obs_local(2)) < (params.corridor_width + radius);

                in_fov = in_dual || in_close;

            otherwise
                error('pathPlanner:badFOV', ...
                    'Unknown fov_mode: %s. Use ''cone'', ''corridor'', or ''dual''.', ...
                    params.fov_mode);
        end

        %% Altitude check: is the drone at the obstacle's altitude?
        altitude_collision = pos(3) >= (z_min - 0.1) && pos(3) <= (z_max + 0.1);

        %% Combine: in FOV + at altitude + within safe distance
        if in_fov && altitude_collision && dist_xy < params.safe_distance
            if dist_xy < minDist
                minDist  = dist_xy;
                obsAngle = obs_global_angle;
                detected = true;

                % Track blocking obstacle's vertical extent
                blocking_z_max = max(blocking_z_max, z_max);
                blocking_z_min = min(blocking_z_min, z_min);
            end
        end
    end

    %% Determine vertical escape options
    if detected && params.enable_altitude
        alt_up   = pos(3) + params.altitude_step;
        alt_down = pos(3) - params.altitude_step;

        canUp   = alt_up   <= params.max_altitude && alt_up   > blocking_z_max + 0.1;
        canDown = alt_down >= params.min_altitude  && alt_down < blocking_z_min - 0.1;
    end
end
