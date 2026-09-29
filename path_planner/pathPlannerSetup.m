function planner = pathPlannerSetup(goal, params)
% PATHPLANNERSETUP  Initialize the obstacle avoidance path planner.
%
%   planner = pathPlannerSetup(goal, params)
%
%   Initializes the wall-follower state machine and configures all
%   avoidance parameters. This function is called ONCE before the
%   simulation loop. Unlike deepcSetup(), there is no heavy precomputation
%   here — this is purely initialization (defaults, state, constants).
%
%   The path planner is a PLANNING layer that generates setpoints r(t).
%   It does NOT contain any controller — controllers live in
%   controllers/ and receive r(t) as input.
%
%   Architecture:
%     pathPlannerSetup()   → creates planner struct (init, called once)
%     pathPlannerStep()    → produces setpoint each timestep (called every tick)
%
%   Inputs:
%       goal   - Goal position [x; y; z] (3x1 vector)
%       params - Configuration struct with fields:
%
%         --- Timing ---
%         .Ts               - Sampling period [s] (default: 0.01)
%
%         --- Vehicle ---
%         .drone_radius     - Collision radius [m] (default: 0.15)
%         .max_yaw_rate     - Maximum yaw rate [rad/s] (default: pi/2)
%
%         --- FOV Detection ---
%         .fov_mode         - 'cone', 'corridor', or 'dual' (default: 'corridor')
%         .fov_angle        - Total FOV angle [rad] (default: deg2rad(90))
%         .detection_range  - Maximum detection range [m] (default: 2.5)
%         .safe_distance    - Minimum safe distance to obstacle [m] (default: 0.8)
%         .corridor_width   - Half-width of corridor FOV [m] (default: 0.4)
%         .corridor_length  - Length of corridor FOV [m] (default: 1.5)
%
%         --- Altitude Control (Parallel FSM) ---
%         .enable_altitude  - Enable 3D altitude avoidance (default: true)
%         .altitude_step    - Altitude change per maneuver [m] (default: 0.5)
%         .min_altitude     - Minimum allowed altitude [m] (default: 0.3)
%         .max_altitude     - Maximum allowed altitude [m] (default: 3.0)
%
%         --- State Machine ---
%         .align_tolerance  - Yaw alignment tolerance [rad] (default: deg2rad(8))
%         .goal_tolerance   - Distance to consider goal reached [m] (default: 0.25)
%         .step_multiplier  - Multiplier for translation step [−] (default: 1.2)
%         .setpoint_gain    - Setpoint advance gain [m] (default: 0.3)
%
%         --- Stuck Detection ---
%         .stuck_threshold  - Steps before declaring stuck [−] (default: 40)
%         .stuck_escape_dist - Lateral escape distance [m] (default: 1.5)
%         .orient_timeout   - Max orient steps before escape [−] (default: 80)
%
%   Output:
%       planner - Struct containing:
%                  .goal           - Goal position (3x1)
%                  .params         - Validated parameters
%                  .state          - Current state struct (mutable)
%                  .constants      - Precomputed constants
%
%   Example:
%       params.Ts = 0.01;
%       params.fov_mode = 'corridor';
%       planner = pathPlannerSetup([10; 10; 1.0], params);
%
%       for k = 1:N
%           [setpoint, yaw, info] = pathPlannerStep(planner, pos, yaw, obstacles);
%           % ... feed setpoint to controller ...
%       end
%
%   Reference: Nycolas Ramires, TCC — Wall-follower state machine (Section 5)

    %% Add functions folder to path
    funcPath = fullfile(fileparts(mfilename('fullpath')), 'functions');
    addpath(funcPath);

    %% Validate goal
    assert(numel(goal) == 3, 'pathPlanner:badGoal', ...
        'Goal must be a 3x1 vector [x; y; z].');
    goal = goal(:);

    %% Set defaults
    params = setDefault(params, 'Ts',              0.01);
    params = setDefault(params, 'drone_radius',    0.15);
    params = setDefault(params, 'max_yaw_rate',    pi/2);  % 90 deg/s

    % FOV detection defaults
    params = setDefault(params, 'fov_mode',        'corridor');
    params = setDefault(params, 'fov_angle',       deg2rad(90));
    params = setDefault(params, 'detection_range', 2.5);
    params = setDefault(params, 'safe_distance',   0.8);
    params = setDefault(params, 'corridor_width',  0.4);
    params = setDefault(params, 'corridor_length', 1.5);

    % Altitude control defaults
    params = setDefault(params, 'enable_altitude',  true);
    params = setDefault(params, 'altitude_step',    0.5);
    params = setDefault(params, 'min_altitude',     0.3);
    params = setDefault(params, 'max_altitude',     3.0);
    params = setDefault(params, 'height_limit_tolerance', 0.005);
    params = setDefault(params, 'altitude_hold_steps', 0);

    % State machine defaults
    params = setDefault(params, 'align_tolerance',  deg2rad(8));
    params = setDefault(params, 'goal_tolerance',   0.25);
    params = setDefault(params, 'step_multiplier',  1.2);
    params = setDefault(params, 'setpoint_gain',    0.3);
    params = setDefault(params, 'min_obstacle_step_dist', 0.5);
    params = setDefault(params, 'min_maneuver_step',      0.6);
    params = setDefault(params, 'max_translate_step',     0.2);
    params = setDefault(params, 'max_escape_step',        0.25);

    % Stuck detection defaults
    params = setDefault(params, 'stuck_threshold',    40);
    params = setDefault(params, 'stuck_escape_dist',  1.5);
    params = setDefault(params, 'orient_timeout',     80);

    %% Derived constants
    constants = struct();
    constants.fov_half = params.fov_angle / 2;

    %% State enumeration (fixed integer IDs)
    %  Lateral FSM states
    constants.ALIGN              = 1;
    constants.GO_TO_GOAL         = 2;
    constants.MANEUVER_ORIENT    = 3;
    constants.MANEUVER_TRANSLATE = 4;
    constants.LATERAL_ESCAPE     = 5;
    constants.AT_GOAL            = 6;
    constants.COLLISION          = 7;

    %  Altitude FSM states (parallel)
    constants.ALT_IDLE           = 0;
    constants.ALT_ASCENDING      = 1;
    constants.ALT_DESCENDING     = 2;
    constants.ALT_HEIGHT_LIMIT   = 3;

    constants.stateNames = {'ALIGN', 'GO_TO_GOAL', 'ORIENT', ...
        'TRANSLATE', 'LAT_ESCAPE', 'AT_GOAL', 'COLLISION'};
    constants.altStateNames = {'IDLE', 'ASCENDING', 'DESCENDING', ...
        'HEIGHT_LIMIT'};

    %% Initialize mutable state
    state = struct();

    % Lateral FSM
    state.lateral        = constants.ALIGN;
    state.prev_lateral   = constants.ALIGN;

    % Altitude FSM (parallel)
    state.altitude       = constants.ALT_IDLE;
    state.altitude_target = goal(3);  % Start targeting goal altitude
    state.altitude_hold_counter = 0;

    % Counters
    state.stuck_counter  = 0;
    state.orient_counter = 0;
    state.prev_dist      = inf;
    state.escape_dir     = 1;  % +1 or -1 for alternating escape side

    % Maneuver data
    state.maneuver_step  = 0;
    state.maneuver_yaw   = 0;  % Target yaw during orient

    % Step counter
    state.step_count     = 0;

    %% Pack planner struct
    planner = struct();
    planner.goal      = goal;
    planner.params    = params;
    planner.state     = state;
    planner.constants = constants;

    %% Print summary
    fprintf('====================================================\n');
    fprintf('  Path Planner Setup (Wall-Follower)\n');
    fprintf('====================================================\n');
    fprintf('  Goal:            [%.2f, %.2f, %.2f]\n', goal);
    fprintf('  FOV mode:        %s\n', params.fov_mode);
    fprintf('  FOV angle:       %.0f°\n', rad2deg(params.fov_angle));
    fprintf('  Detection range: %.2f m\n', params.detection_range);
    fprintf('  Safe distance:   %.2f m\n', params.safe_distance);
    fprintf('  Altitude ctrl:   %s\n', boolStr(params.enable_altitude));
    fprintf('  Stuck threshold: %d steps\n', params.stuck_threshold);
    fprintf('====================================================\n');

end

%% Helper: set default value if field is missing
function s = setDefault(s, field, value)
    if ~isfield(s, field)
        s.(field) = value;
    end
end

%% Helper: boolean to string
function s = boolStr(b)
    if b, s = 'enabled'; else, s = 'disabled'; end
end
