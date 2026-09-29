function obstacles = createObstacleEnvironment(preset, customParams)
% CREATEOBSTACLEENVIRONMENT  Generate obstacle lists for testing.
%
%   obstacles = createObstacleEnvironment(preset)
%   obstacles = createObstacleEnvironment('custom', customParams)
%
%   Each row of obstacles: [x, y, z_min, z_max, radius]
%
%   Presets:
%       'simple'       - 3 obstacles, easy navigation
%       'corridor'     - Linear corridor with narrow passage
%       'dense'        - Dense obstacle field (10+ obstacles)
%       'mixed_3d'     - Mix of low, tall, and floating obstacles
%       'challenging'  - Complex scenario with dead-ends
%       'custom'       - User-defined obstacles via customParams
%
%   For 'custom', customParams must contain:
%       .obstacles  - N x 5 matrix [x, y, z_min, z_max, radius]
%
%   Example:
%       obs = createObstacleEnvironment('simple');
%       obs = createObstacleEnvironment('custom', struct('obstacles', my_obs));

    if nargin < 2
        customParams = struct();
    end

    switch preset
        case 'simple'
            obstacles = [
                3.0, 3.0, 0, 2.5, 0.5;
                5.0, 5.0, 0, 2.5, 0.6;
                7.0, 4.0, 0, 2.5, 0.4;
            ];

        case 'corridor'
            obstacles = [
                % Left wall
                2.0, 1.0, 0, 2.5, 0.3;
                2.0, 3.0, 0, 2.5, 0.3;
                2.0, 5.0, 0, 2.5, 0.3;
                2.0, 7.0, 0, 2.5, 0.3;
                % Right wall with gap
                4.0, 1.0, 0, 2.5, 0.3;
                4.0, 3.0, 0, 2.5, 0.3;
                % Gap at y=5.0
                4.0, 7.0, 0, 2.5, 0.3;
            ];

        case 'dense'
            rng(42);
            n = 12;
            obstacles = zeros(n, 5);
            for i = 1:n
                obstacles(i, :) = [
                    2 + rand()*6, ...      % x
                    2 + rand()*6, ...      % y
                    0, ...                  % z_min
                    2.5, ...               % z_max
                    0.3 + rand()*0.4       % radius
                ];
            end

        case 'mixed_3d'
            obstacles = [
                % Standard full-height cylinders
                2.5, 2.0, 0, 2.5, 0.5;
                5.0, 4.5, 0, 2.5, 0.6;
                6.0, 2.5, 0, 2.5, 0.5;
                7.0, 6.0, 0, 2.5, 0.45;
                % Low obstacle (can fly OVER at z > 0.8)
                4.0, 3.5, 0, 0.7, 0.6;
                % Floating obstacle (can fly UNDER at z < 1.8)
                6.5, 5.0, 1.8, 2.5, 0.5;
                % Mid-height obstacle
                3.0, 5.5, 0, 1.5, 0.4;
            ];

        case 'challenging'
            obstacles = [
                % Outer barrier (almost encircling)
                2.0, 2.0, 0, 2.5, 0.4;
                3.5, 2.0, 0, 2.5, 0.4;
                5.0, 2.0, 0, 2.5, 0.4;
                6.5, 2.5, 0, 2.5, 0.4;
                7.0, 4.0, 0, 2.5, 0.4;
                7.0, 5.5, 0, 2.5, 0.4;
                % Inner obstacles
                4.0, 4.0, 0, 2.5, 0.5;
                5.0, 5.5, 0, 2.5, 0.5;
                % Low obstacle forming a "trap" (escapable via altitude)
                3.5, 5.0, 0, 0.8, 0.7;
                5.5, 3.5, 0, 0.8, 0.7;
            ];

        case 'custom'
            assert(isfield(customParams, 'obstacles'), ...
                'pathPlanner:noObstacles', ...
                'customParams.obstacles required for ''custom'' preset.');
            obstacles = customParams.obstacles;

        otherwise
            error('pathPlanner:unknownPreset', ...
                'Unknown preset: %s. Options: simple, corridor, dense, mixed_3d, challenging, custom.', preset);
    end

    fprintf('  Obstacle environment: %s (%d obstacles)\n', preset, size(obstacles, 1));
end
