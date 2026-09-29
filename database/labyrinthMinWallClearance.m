function [clearance, sampleIdx, wallIdx] = labyrinthMinWallClearance(traj, s)
% LABYRINTHMINWALLCLEARANCE Minimum drone-body clearance to labyrinth walls.
%
%   traj must be N x 3 or N x 2. Clearance is distance to the nearest wall
%   segment minus the drone radius. Negative means wall collision.

    walls = labyrinthWallSegments(s);
    clearance = inf;
    sampleIdx = 0;
    wallIdx = 0;

    for k = 1:size(traj, 1)
        p = traj(k, 1:2);
        for i = 1:size(walls, 1)
            a = walls(i, 1:2);
            b = walls(i, 3:4);
            d = b - a;
            alpha = dot(p - a, d) / max(dot(d, d), 1e-12);
            alpha = max(0, min(1, alpha));
            closest = a + alpha * d;
            c = norm(p - closest) - s.droneRadius;

            if c < clearance
                clearance = c;
                sampleIdx = k;
                wallIdx = i;
            end
        end
    end
end
