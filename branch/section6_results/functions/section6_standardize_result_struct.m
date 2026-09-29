function result = section6_standardize_result_struct(rawLog, method, scenarioName, scenarioData)
% SECTION6_STANDARDIZE_RESULT_STRUCT Convert raw log to standardized struct.
% rawLog: result.log from DeePC or mpc_data.obstacleResults.iterativeResult.log
% method: 'DeePC' or 'MPC'
% scenarioName: e.g., 'Scenario A - No Obstacles'
% scenarioData: the full scenario struct containing obstacles/walls

    result = struct();
    result.method = method;
    result.scenario_name = scenarioName;
    
    % Time vector
    if isfield(rawLog, 'time')
        result.t = rawLog.time(:);
    else
        result.t = [];
    end
    
    % Reference trajectory
    if isfield(rawLog, 'ref')
        r = rawLog.ref;
        if size(r, 1) == 3 && size(r, 2) > 3
            result.ref.position = r';
        else
            result.ref.position = r;
        end
    else
        result.ref.position = [];
    end
    
    % Actual position
    if isfield(rawLog, 'pos')
        p = rawLog.pos;
        if size(p, 1) == 3 && size(p, 2) > 3
            result.actual.position = p';
        else
            result.actual.position = p;
        end
    else
        result.actual.position = [];
    end
    
    % Euler angles
    if isfield(rawLog, 'euler')
        e = rawLog.euler;
        if size(e, 1) == 3 && size(e, 2) > 3
            result.actual.euler = e';
        else
            result.actual.euler = e;
        end
    else
        result.actual.euler = []; % DeePC lacks euler
    end
    
    % Motor inputs
    if isfield(rawLog, 'u')
        u = rawLog.u;
        if size(u, 1) == 4 && size(u, 2) > 4
            result.control.motors = u';
        else
            result.control.motors = u;
        end
    else
        result.control.motors = [];
    end
    
    % Safety margin
    if isfield(rawLog, 'safetyMargin')
        sm = rawLog.safetyMargin;
        if isfield(sm, 'margin')
            result.safety.h = sm.margin(:);
        else
            result.safety.h = [];
        end
        if isfield(sm, 'obsClearance')
            result.safety.distance = sm.obsClearance(:);
        else
            result.safety.distance = [];
        end
    else
        result.safety.h = [];
        result.safety.distance = [];
    end
    
    % State Machine
    if isfield(rawLog, 'constraintCount')
        % Construct mock state machine: 0 = tracking, 4 = constraints active
        result.state_machine.state_id = zeros(size(rawLog.constraintCount));
        result.state_machine.state_id(rawLog.constraintCount > 0) = 4; % Predictive Optimization / Avoidance
        result.state_machine.state_name = {'Tracking', 'Avoidance'};
    else
        result.state_machine.state_id = [];
        result.state_machine.state_name = {};
    end
    
    % Obstacles
    if nargin >= 4 && isfield(scenarioData, 'obstacles')
        result.obstacles = scenarioData.obstacles;
    else
        result.obstacles = [];
    end
    
    if nargin >= 4 && isfield(scenarioData, 'walls')
        result.walls = scenarioData.walls;
    else
        result.walls = [];
    end
    
    % Metrics placeholder
    result.metrics = struct();
    result.notes = '';
end
