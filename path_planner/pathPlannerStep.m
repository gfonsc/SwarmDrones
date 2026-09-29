function [setpoint, yaw_cmd, planner, info] = pathPlannerStep(planner, pos, yaw, obstacles)
% PATHPLANNERSTEP  Online step of the path planner — one tick of the FSM.
%
%   [setpoint, yaw_cmd, planner, info] = pathPlannerStep(planner, pos, yaw, obstacles)
%
%   Executes one timestep of the wall-follower state machine with parallel
%   altitude control. Produces a position setpoint and yaw command for the
%   controller layer.
%
%   This function is the ONLY interface between the path planner and the
%   controller. The controller receives setpoint r(t) and tracks it using
%   its own control law (DeePC, MPC, LQR, or PID).
%
%   Inputs:
%       planner   - Planner struct from pathPlannerSetup() (modified in-place)
%       pos       - Current drone position [x; y; z] (3x1)
%       yaw       - Current drone yaw angle [rad] (scalar)
%       obstacles - Obstacle list (N x 5 matrix):
%                   Each row: [x, y, z_min, z_max, radius]
%                   For 2D obstacles, set z_min=0, z_max=inf
%
%   Outputs:
%       setpoint  - Target position [x; y; z] (3x1) for the controller
%       yaw_cmd   - Target yaw angle [rad] (scalar)
%       planner   - Updated planner struct (pass back next step)
%       info      - Diagnostic struct:
%                    .lateral_state     - Current lateral state ID
%                    .altitude_state    - Current altitude state ID
%                    .lateral_name      - Current lateral state name
%                    .altitude_name     - Current altitude state name
%                    .obstacle_detected - Boolean
%                    .obstacle_dist     - Distance to nearest obstacle
%                    .obstacle_angle    - Angle to nearest obstacle
%                    .can_go_up         - Altitude escape upward possible
%                    .can_go_down       - Altitude escape downward possible
%                    .dist_to_goal      - Distance to goal
%                    .is_stuck          - Boolean
%                    .collision         - Boolean
%                    .step_count        - Cumulative step counter
%
%   Example:
%       planner = pathPlannerSetup(goal, params);
%       for k = 1:N
%           [setpoint, yaw_cmd, planner, info] = pathPlannerStep(planner, pos, yaw, obs);
%           % Feed setpoint to controller:
%           [uOpt, ~, ~] = deepcAlgorithm(model, uIni, yIni, setpoint);
%       end
%
%   Reference: Nycolas Ramires, TCC — Wall-follower (Section 5)
%              Proposals 1 & 2 from Section 7.1

    %% Unpack
    goal   = planner.goal;
    p      = planner.params;
    c      = planner.constants;
    s      = planner.state;

    s.step_count = s.step_count + 1;

    %% === COLLISION CHECK ===
    [has_collision, ~] = checkCollision(pos, obstacles, p.drone_radius);

    if has_collision && s.lateral ~= c.COLLISION
        s.lateral = c.COLLISION;
    end

    %% === OBSTACLE DETECTION ===
    [obs_det, obs_dist, obs_angle, can_up, can_down] = ...
        detectObstacles(pos, yaw, obstacles, p, c);

    %% === DISTANCE TO GOAL ===
    dist_to_goal = norm(pos - goal);

    %% === STUCK DETECTION ===
    if dist_to_goal >= s.prev_dist - 0.002
        s.stuck_counter = s.stuck_counter + 1;
    else
        s.stuck_counter = max(0, s.stuck_counter - 3);
    end
    s.prev_dist = dist_to_goal;
    is_stuck = s.stuck_counter > p.stuck_threshold;

    %% === PARALLEL ALTITUDE FSM ===
    %  Runs independently of the lateral FSM.
    %  Nycolas §7.1: "could be implemented in another state machine
    %  that would operate in parallel to the setpoint generator"
    altitude_target = s.altitude_target;

    if p.enable_altitude && pos(3) >= p.max_altitude - p.height_limit_tolerance
        s.altitude = c.ALT_HEIGHT_LIMIT;
        altitude_target = max(pos(3) - p.altitude_step, p.min_altitude);
    end

    switch s.altitude
        case c.ALT_IDLE
            % Check if altitude change is beneficial
            if obs_det && p.enable_altitude
                if can_up
                    s.altitude = c.ALT_ASCENDING;
                    altitude_target = min(pos(3) + p.altitude_step, p.max_altitude);
                    s.altitude_hold_counter = p.altitude_hold_steps;
                elseif can_down
                    s.altitude = c.ALT_DESCENDING;
                    altitude_target = max(pos(3) - p.altitude_step, p.min_altitude);
                    s.altitude_hold_counter = p.altitude_hold_steps;
                end
            else
                altitude_target = goal(3);
            end

        case c.ALT_ASCENDING
            altitude_target = s.altitude_target;
            if abs(pos(3) - altitude_target) < 0.1 && s.altitude_hold_counter > 0
                s.altitude_hold_counter = s.altitude_hold_counter - 1;
            elseif abs(pos(3) - altitude_target) < 0.1
                s.altitude = c.ALT_IDLE;
            end

        case c.ALT_DESCENDING
            altitude_target = s.altitude_target;
            if abs(pos(3) - altitude_target) < 0.1 && s.altitude_hold_counter > 0
                s.altitude_hold_counter = s.altitude_hold_counter - 1;
            elseif abs(pos(3) - altitude_target) < 0.1
                s.altitude = c.ALT_IDLE;
            end

        case c.ALT_HEIGHT_LIMIT
            altitude_target = max(pos(3) - p.altitude_step, p.min_altitude);
            if pos(3) < p.max_altitude - 2 * p.height_limit_tolerance
                s.altitude = c.ALT_IDLE;
            end
    end

    s.altitude_target = altitude_target;

    %% === LATERAL FSM ===
    %  The main wall-follower state machine.
    %  States: ALIGN → GO_TO_GOAL → ORIENT → TRANSLATE → (back to ALIGN)
    %  Priority: Obstacle > !Aligned > Arrived

    setpoint = pos;     % Default: hold position
    yaw_cmd  = yaw;     % Default: hold yaw

    switch s.lateral
        % ----- ALIGN: Rotate yaw to face the goal -----
        case c.ALIGN
            s.orient_counter = 0;
            target_yaw = atan2(goal(2) - pos(2), goal(1) - pos(1));
            yaw_error  = wrapToPi(target_yaw - yaw);

            if abs(yaw_error) < p.align_tolerance
                % Aligned — decide next state
                if obs_det
                    s.lateral = c.MANEUVER_ORIENT;
                    s.maneuver_step = p.step_multiplier * ...
                        max(obs_dist, p.min_obstacle_step_dist);
                else
                    s.lateral = c.GO_TO_GOAL;
                end
            else
                % Rotate toward goal
                yaw_step = sign(yaw_error) * min(abs(yaw_error), p.max_yaw_rate * p.Ts);
                yaw_cmd  = yaw + yaw_step;
            end

            setpoint = pos;
            setpoint(3) = altitude_target;

        % ----- GO_TO_GOAL: Advance toward goal -----
        case c.GO_TO_GOAL
            s.orient_counter = 0;

            if dist_to_goal < p.goal_tolerance
                s.lateral = c.AT_GOAL;

            elseif is_stuck
                s.lateral = c.LATERAL_ESCAPE;
                s.escape_dir = -s.escape_dir;
                s.maneuver_step = p.stuck_escape_dist;
                s.stuck_counter = 0;

            elseif obs_det
                s.lateral = c.MANEUVER_ORIENT;
                s.maneuver_step = p.step_multiplier * ...
                    max(obs_dist, p.min_obstacle_step_dist);

            else
                % Advance toward goal
                dir = (goal - pos);
                dir = dir / (norm(dir) + 1e-8);

                % Setpoint step function inspired by Nycolas Eq. 9:
                %   f_step = 2.56 * atan(0.5 * d)
                % Simplified to a bounded linear advance:
                advance = min(p.setpoint_gain, dist_to_goal * 0.4);
                setpoint = pos + dir * advance;
                setpoint(3) = altitude_target;

                % Keep yaw aligned to movement direction
                yaw_cmd = atan2(dir(2), dir(1));
            end

        % ----- ORIENT: Yaw to clear obstacle from FOV -----
        case c.MANEUVER_ORIENT
            s.orient_counter = s.orient_counter + 1;

            if s.orient_counter > p.orient_timeout
                % Force lateral escape
                s.lateral = c.LATERAL_ESCAPE;
                s.escape_dir = -s.escape_dir;
                s.maneuver_step = p.stuck_escape_dist;
                s.orient_counter = 0;

            elseif ~obs_det
                % Obstacle cleared from FOV → advance
                s.lateral = c.MANEUVER_TRANSLATE;
                s.maneuver_step = max(p.min_maneuver_step, s.maneuver_step);

            else
                % Rotate away from obstacle
                rot_dir = sign(obs_angle);
                if rot_dir == 0
                    rot_dir = s.escape_dir;
                end
                yaw_cmd = yaw - rot_dir * p.max_yaw_rate * p.Ts;
            end

            setpoint = pos;
            setpoint(3) = altitude_target;

        % ----- TRANSLATE: Move forward past the obstacle -----
        case c.MANEUVER_TRANSLATE
            if s.maneuver_step <= 0
                s.lateral = c.ALIGN;

            elseif obs_det
                % New obstacle ahead — re-orient
                s.lateral = c.MANEUVER_ORIENT;

            else
                forward = [cos(yaw); sin(yaw); 0];
                move = min(p.max_translate_step, s.maneuver_step);
                setpoint = pos + forward * move;
                setpoint(3) = altitude_target;
                s.maneuver_step = s.maneuver_step - move;

                yaw_cmd = yaw;  % Maintain heading
            end

        % ----- LATERAL ESCAPE: Move perpendicular to unblock -----
        case c.LATERAL_ESCAPE
            goal_dir   = atan2(goal(2) - pos(2), goal(1) - pos(1));
            escape_yaw = goal_dir + s.escape_dir * pi/2;

            if s.maneuver_step <= 0
                s.lateral = c.ALIGN;

            else
                forward = [cos(escape_yaw); sin(escape_yaw); 0];
                move = min(p.max_escape_step, s.maneuver_step);
                setpoint = pos + forward * move;
                setpoint(3) = altitude_target;
                s.maneuver_step = s.maneuver_step - move;

                yaw_cmd = escape_yaw;
            end

        % ----- AT GOAL: Hold position -----
        case c.AT_GOAL
            setpoint = goal;
            yaw_cmd  = yaw;

        % ----- COLLISION: Emergency stop -----
        case c.COLLISION
            setpoint = pos;
            yaw_cmd  = yaw;
    end

    %% Clamp altitude
    setpoint(3) = max(p.min_altitude, min(p.max_altitude, setpoint(3)));

    %% Update state
    planner.state = s;

    %% Build diagnostic info
    info = struct();
    info.lateral_state     = s.lateral;
    info.altitude_state    = s.altitude;
    info.lateral_name      = c.stateNames{s.lateral};
    info.altitude_name     = c.altStateNames{s.altitude + 1};
    info.obstacle_detected = obs_det;
    info.obstacle_dist     = obs_dist;
    info.obstacle_angle    = obs_angle;
    info.can_go_up         = can_up;
    info.can_go_down       = can_down;
    info.height_limit      = (s.altitude == c.ALT_HEIGHT_LIMIT);
    info.dist_to_goal      = dist_to_goal;
    info.is_stuck          = is_stuck;
    info.collision         = has_collision;
    info.step_count        = s.step_count;
end

%% ========================================================================
%  HELPER: Wrap angle to [-pi, pi]
%  ========================================================================
function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
