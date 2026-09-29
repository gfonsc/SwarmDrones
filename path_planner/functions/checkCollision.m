function [collision, obsIdx] = checkCollision(pos, obstacles, droneRadius)
% CHECKCOLLISION  Check if the drone has collided with any obstacle.
%
%   [collision, obsIdx] = checkCollision(pos, obstacles, droneRadius)
%
%   A collision occurs when the XY distance between the drone center and
%   an obstacle center is less than the sum of their radii, AND the drone
%   is at the obstacle's altitude range.
%
%   Inputs:
%       pos        - Drone position [x; y; z] (3x1)
%       obstacles  - Obstacle matrix (N x 5): [x, y, z_min, z_max, radius]
%       droneRadius - Drone collision radius [m]
%
%   Outputs:
%       collision  - true if collision detected
%       obsIdx     - Index of the collided obstacle (0 if none)

    collision = false;
    obsIdx    = 0;

    if isempty(obstacles)
        return;
    end

    for i = 1:size(obstacles, 1)
        ox     = obstacles(i, 1);
        oy     = obstacles(i, 2);
        z_min  = obstacles(i, 3);
        z_max  = obstacles(i, 4);
        radius = obstacles(i, 5);

        dist_xy = sqrt((pos(1) - ox)^2 + (pos(2) - oy)^2);

        if dist_xy < (radius + droneRadius) && ...
           pos(3) > z_min && pos(3) < z_max
            collision = true;
            obsIdx    = i;
            return;
        end
    end
end
