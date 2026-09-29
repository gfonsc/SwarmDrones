function s = labyrinthScenarioConfig()
% LABYRINTHSCENARIOCONFIG Shared geometry and envelope for DeePC labyrinth tests.

    s = struct();
    s.name = 'LabyrinthDeePCMission';

    s.A = [0, 0, 0];
    s.zHover = 1.0;
    s.zCeiling = 1.85;
    s.droneRadius = 0.08;
    s.safeDistance = 0.18;
    s.corridorHalfWidth = 1.00;
    s.dt = 0.10;

    s.bounds.x = [-0.8, 8.6];
    s.bounds.y = [-0.8, 6.8];
    s.bounds.z = [0.0, s.zCeiling - 0.03];
    s.axis2D = [-1.1, 8.8, -0.9, 6.9];
    s.axis3D = [-1.1, 8.8, -0.9, 6.9, 0.0, 2.05];

    s.centerline = [
        0.0, -0.8;
        0.0, 0.0;
        0.0, 6.0;
        2.0, 6.0;
        2.0, 0.6;
        4.0, 0.6;
        4.0, 6.0;
        6.0, 6.0;
        6.0, 0.6;
        8.0, 0.6;
        8.8, 0.6
    ];

    s.missionSetpoints = [
        0.00, 0.00, 0.00;
        0.00, 0.00, 1.00;
        0.00, 6.00, 1.00;
        2.00, 6.00, 1.00;
        2.00, 0.60, 1.00;
        4.00, 0.60, 1.00;
        4.00, 6.00, 1.00;
        6.00, 6.00, 1.00;
        6.00, 0.60, 1.00;
        8.00, 0.60, 1.00
    ];
    s.setpoints = s.missionSetpoints;
    s.B = s.missionSetpoints(end, :);

    % Dense excitation route used only to collect DeePC data. The online
    % demo should use missionSetpoints plus the path planner, not these
    % pre-shaped avoidance points.
    s.trainingSetpoints = [
        0.00, 0.00, 0.00;
        0.00, 0.00, 1.00;
        0.00, 1.45, 1.00;
        0.68, 2.30, 1.00;
        0.68, 3.75, 1.00;
        0.00, 4.45, 1.00;
        0.00, 6.00, 1.00;
        2.00, 6.00, 1.00;
        2.00, 4.15, 1.45;
        2.00, 1.35, 1.45;
        2.00, 0.60, 1.00;
        4.00, 0.60, 1.00;
        4.00, 2.75, 0.55;
        4.00, 5.05, 0.55;
        4.00, 6.00, 1.00;
        6.00, 6.00, 1.00;
        6.00, 4.55, 1.00;
        5.45, 3.70, 1.00;
        5.45, 2.55, 1.00;
        6.00, 1.70, 1.00;
        6.00, 0.60, 1.00;
        8.00, 0.60, 1.00
    ];

    % Cylinders: [x, y, z_min, z_max, radius, mode_id].
    % mode_id: 1 lateral right, 2 altitude up, 3 altitude down, 4 lateral left.
    s.obstacles = [
        0.00, 3.00, 0.00, 1.70, 0.28, 1;
        2.00, 3.00, 0.00, 1.20, 0.50, 2;
        4.00, 3.55, 1.05, 1.85, 0.70, 3;
        6.00, 3.15, 0.00, 1.70, 0.28, 4
    ];

    % Explicit physical walls. Internal walls are real collision geometry,
    % while gaps at the top/bottom preserve the serpentine maze route.
    % Each row is [x1, y1, x2, y2].
    s.wallSegments = [
       -1.00, -0.80, -1.00,  7.00;
       -1.00,  7.00,  9.00,  7.00;
        9.00,  7.00,  9.00, -0.80;
        9.00, -0.80, -1.00, -0.80;
        1.00, -0.80,  1.00,  5.20;
        3.00,  1.40,  3.00,  7.00;
        5.00, -0.80,  5.00,  5.20;
        7.00,  1.40,  7.00,  7.00
    ];

    s.inputNames = {'vx_cmd', 'vy_cmd', 'vz_cmd', 'yaw_rate_cmd'};
    s.outputNames = {'x', 'y', 'z', 'vx', 'vy', 'vz'};
    s.outputIdx = 1:6;

    s.model = struct();
    s.model.dt = s.dt;
    s.model.tauVel = 0.28;
    s.model.tauYaw = 0.22;
    s.model.uMin = [-1.35; -1.35; -0.75; -1.50];
    s.model.uMax = [ 1.35;  1.35;  0.75;  1.50];
    s.model.stateMin = [s.bounds.x(1); s.bounds.y(1); s.bounds.z(1); ...
        -inf; -inf; -inf; -pi; -inf];
    s.model.stateMax = [s.bounds.x(2); s.bounds.y(2); s.bounds.z(2); ...
        inf; inf; inf; pi; inf];

    s.dynamicInputNames = {'T', 'tau_x', 'tau_y', 'tau_z'};
    s.dynamicOutputNames = {'x', 'y', 'z', 'vx', 'vy', 'vz', ...
        'roll', 'pitch', 'yaw', 'wx', 'wy', 'wz'};

    s.dynamic = struct();
    s.dynamic.dt = 0.08;
    s.dynamic.mass = 0.045;
    s.dynamic.g = 9.81;
    s.dynamic.J = diag([2.3951e-5, 2.3951e-5, 3.2347e-5]);
    s.dynamic.hoverThrust = s.dynamic.mass * s.dynamic.g;
    s.dynamic.uMin = [0.24; -5.0e-5; -5.0e-5; -2.5e-5];
    s.dynamic.uMax = [0.74;  5.0e-5;  5.0e-5;  2.5e-5];
    s.dynamic.stateMin = [s.bounds.x(1); s.bounds.y(1); s.bounds.z(1); ...
        -2.4; -2.4; -1.5; -0.60; -0.60; -pi; -4.5; -4.5; -3.2];
    s.dynamic.stateMax = [s.bounds.x(2); s.bounds.y(2); s.bounds.z(2); ...
        2.4; 2.4; 1.5; 0.60; 0.60; pi; 4.5; 4.5; 3.2];
end
