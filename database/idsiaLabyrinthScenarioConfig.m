function s = idsiaLabyrinthScenarioConfig()
% IDSIALABYRINTHSCENARIOCONFIG Labyrinth scaled to the IDSIA flight envelope.
%
% The IDSIA benchmark trajectories live roughly inside a two-meter indoor
% volume. This scenario keeps the maze, obstacles, takeoff, hover, lateral
% maneuvers, and altitude maneuvers inside that experimentally observed
% envelope so DeePC is not asked to extrapolate to the large lab maze.

    s = struct();
    s.name = 'IDSIACompactLabyrinth';

    s.A = [-0.65, -0.58, 0.00];
    s.zHover = 1.00;
    s.zCeiling = 2.00;
    s.droneRadius = 0.035;
    s.safeDistance = 0.10;
    s.dt = 0.04;

    s.bounds.x = [-0.82, 0.88];
    s.bounds.y = [-0.74, 0.74];
    s.bounds.z = [0.00, s.zCeiling - 0.03];
    s.axis2D = [-0.88, 0.94, -0.80, 0.80];
    s.axis3D = [-0.88, 0.94, -0.80, 0.80, 0.00, 2.08];

    s.centerline = [
       -0.65, -0.58;
       -0.65,  0.56;
       -0.25,  0.56;
       -0.25, -0.50;
        0.15, -0.50;
        0.15,  0.56;
        0.52,  0.56;
        0.52, -0.50;
        0.74, -0.50
    ];

    s.missionSetpoints = [
       -0.65, -0.58, 0.00;
       -0.65, -0.58, 1.00;
       -0.65,  0.56, 1.00;
       -0.25,  0.56, 1.00;
       -0.25, -0.50, 1.00;
        0.15, -0.50, 1.00;
        0.15,  0.56, 1.00;
        0.52,  0.56, 1.00;
        0.52, -0.50, 1.00;
        0.74, -0.50, 1.00
    ];
    s.setpoints = s.missionSetpoints;
    s.B = s.missionSetpoints(end, :);

    % Cylinders: [x, y, z_min, z_max, radius, mode_id].
    % mode_id: 2 altitude up, 3 altitude down. The blue obstacle is attached
    % to the floor and rises 1.6 m; the orange obstacle is attached to the
    % 2 m ceiling and descends 1.6 m.
    s.obstacles = [
       -0.25,  0.02, 0.00, 1.60, 0.108, 2;
        0.15,  0.10, 0.40, 2.00, 0.130, 3
    ];

    % Explicit physical walls. Each row is [x1, y1, x2, y2].
    s.wallSegments = [
       -0.78, -0.72, -0.78,  0.72;
       -0.78,  0.72,  0.86,  0.72;
        0.86,  0.72,  0.86, -0.72;
        0.86, -0.72, -0.78, -0.72;
       -0.45, -0.72, -0.45,  0.38;
       -0.05, -0.30, -0.05,  0.72;
        0.34, -0.72,  0.34,  0.38;
        0.64, -0.30,  0.64,  0.72
    ];

    s.dynamicInputNames = {'T', 'tau_x', 'tau_y', 'tau_z'};
    s.dynamicOutputNames = {'x', 'y', 'z', 'vx', 'vy', 'vz', ...
        'roll', 'pitch', 'yaw', 'wx', 'wy', 'wz'};
end
