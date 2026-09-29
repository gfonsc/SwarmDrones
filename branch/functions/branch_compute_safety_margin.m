function [h, details] = branch_compute_safety_margin(traj, scenario, varargin)
%BRANCH_COMPUTE_SAFETY_MARGIN Compute signed safety margin to the closest wall or obstacle.

    p = inputParser;
    addParameter(p, 'SafeDistance', scenario.safeDistance, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'UseWalls', true, @islogical);
    addParameter(p, 'UseObstacles', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    traj = coerceTrajectory(traj);
    n = size(traj, 1);

    safeClearance = scenario.droneRadius + opts.SafeDistance;
    bodyClearance = scenario.droneRadius;

    h = inf(n, 1);
    nearestIndex = zeros(n, 1);
    nearestSource = repmat({'none'}, n, 1);
    nearestNormal = zeros(n, 3);
    nearestBoundary = nan(n, 3);
    obstacleMargin = inf(n, 1);
    wallMargin = inf(n, 1);
    obstacleClearance = inf(n, 1);
    wallClearance = inf(n, 1);

    if opts.UseWalls
        walls = labyrinthWallSegments(scenario);
    else
        walls = zeros(0, 4);
    end

    if opts.UseObstacles
        obstacles = scenario.obstacles;
    else
        obstacles = zeros(0, max(5, size(scenario.obstacles, 2)));
    end

    for k = 1:n
        point = traj(k, :);

        bestMargin = inf;
        bestSource = 'none';
        bestIndex = 0;
        bestNormal = [0, 0, 0];
        bestBoundary = [nan, nan, nan];

        for i = 1:size(obstacles, 1)
            [marginObs, normalObs, boundaryObs] = signedDistanceToInflatedCylinder( ...
                point, obstacles(i, :), safeClearance, bodyClearance);
            rawObs = signedDistanceToPhysicalCylinder(point, obstacles(i, :), bodyClearance);
            obstacleMargin(k) = min(obstacleMargin(k), marginObs);
            obstacleClearance(k) = min(obstacleClearance(k), rawObs);

            if marginObs < bestMargin
                bestMargin = marginObs;
                bestSource = 'obstacle';
                bestIndex = i;
                bestNormal = normalObs;
                bestBoundary = boundaryObs;
            end
        end

        for i = 1:size(walls, 1)
            [marginWall, normalWall, boundaryWall] = signedDistanceToInflatedWall( ...
                point, walls(i, :), safeClearance);
            rawWall = signedDistanceToWall(point, walls(i, :), bodyClearance);
            wallMargin(k) = min(wallMargin(k), marginWall);
            wallClearance(k) = min(wallClearance(k), rawWall);

            if marginWall < bestMargin
                bestMargin = marginWall;
                bestSource = 'wall';
                bestIndex = i;
                bestNormal = normalWall;
                bestBoundary = boundaryWall;
            end
        end

        h(k) = bestMargin;
        nearestIndex(k) = bestIndex;
        nearestSource{k} = bestSource;
        nearestNormal(k, :) = bestNormal;
        nearestBoundary(k, :) = bestBoundary;
    end

    details = struct();
    details.safeClearance = safeClearance;
    details.bodyClearance = bodyClearance;
    details.nearestIndex = nearestIndex;
    details.nearestSource = nearestSource;
    details.nearestNormal = nearestNormal;
    details.nearestBoundary = nearestBoundary;
    details.obstacleMargin = obstacleMargin;
    details.wallMargin = wallMargin;
    details.obstacleClearance = obstacleClearance;
    details.wallClearance = wallClearance;
end

function traj = coerceTrajectory(traj)
    if isempty(traj)
        traj = zeros(0, 3);
        return;
    end
    if size(traj, 2) == 3
        return;
    end
    if size(traj, 1) == 3
        traj = traj';
        return;
    end
    error('branch:badTrajectoryShape', 'Trajectory must be N x 3 or 3 x N.');
end

function [margin, normal, boundary] = signedDistanceToInflatedCylinder(point, obs, safeClearance, bodyClearance)
    zMin = obs(3);
    zMax = obs(4);
    if numel(obs) < 5
        error('branch:badObstacle', 'Obstacle rows must contain at least five elements.');
    end

    inflatedRadius = obs(5) + safeClearance;
    physicalRadius = obs(5) + bodyClearance;

    centerXY = obs(1:2);
    deltaXY = point(1:2) - centerXY;
    radialDist = norm(deltaXY);

    radialMargin = radialDist - inflatedRadius;
    dzBelow = zMin - point(3);
    dzAbove = point(3) - zMax;
    verticalGap = max([dzBelow, dzAbove, 0]);

    if verticalGap == 0
        margin = radialMargin;
        if radialDist > 1e-9
            normal = [deltaXY / radialDist, 0];
        else
            normal = fallbackObstacleNormal(obs);
        end
    elseif radialMargin > 0
        cornerVector = [max(radialMargin, 0) * radialUnit(deltaXY, obs), verticalSign(dzBelow, dzAbove) * verticalGap];
        cornerNorm = norm(cornerVector);
        if cornerNorm > 1e-9
            normal = cornerVector / cornerNorm;
        else
            normal = fallbackObstacleNormal(obs);
        end
        margin = hypot(max(radialMargin, 0), verticalGap);
    else
        margin = verticalGap;
        normal = [0, 0, verticalSign(dzBelow, dzAbove)];
    end

    insideVertical = (point(3) >= zMin) && (point(3) <= zMax);
    if insideVertical && radialDist < inflatedRadius
        radialPen = inflatedRadius - radialDist;
        upGap = zMax - point(3);
        downGap = point(3) - zMin;
        [minGap, idx] = min([radialPen, upGap, downGap]);
        margin = -minGap;
        switch idx
            case 1
                if radialDist > 1e-9
                    normal = [deltaXY / radialDist, 0];
                else
                    normal = fallbackObstacleNormal(obs);
                end
            case 2
                normal = [0, 0, 1];
            case 3
                normal = [0, 0, -1];
        end
    end

    boundary = point - margin * normal;

    if radialDist < physicalRadius && insideVertical
        rawMargin = radialDist - physicalRadius;
    elseif insideVertical
        rawMargin = radialDist - physicalRadius;
    elseif radialDist <= physicalRadius
        rawMargin = verticalGap;
    else
        rawMargin = hypot(max(radialDist - physicalRadius, 0), verticalGap);
    end

    if nargout > 3 %#ok<UNRCH>
        boundary = rawMargin; % compatibility guard
    end
end

function rawMargin = signedDistanceToPhysicalCylinder(point, obs, bodyClearance)
    zMin = obs(3);
    zMax = obs(4);
    physicalRadius = obs(5) + bodyClearance;
    deltaXY = point(1:2) - obs(1:2);
    radialDist = norm(deltaXY);
    radialMargin = radialDist - physicalRadius;
    dzBelow = zMin - point(3);
    dzAbove = point(3) - zMax;
    verticalGap = max([dzBelow, dzAbove, 0]);
    insideVertical = (point(3) >= zMin) && (point(3) <= zMax);

    if insideVertical
        rawMargin = radialMargin;
        return;
    end

    if radialMargin <= 0
        rawMargin = verticalGap;
    else
        rawMargin = hypot(radialMargin, verticalGap);
    end
end

function [margin, normal, boundary] = signedDistanceToInflatedWall(point, wall, safeClearance)
    a = wall(1:2);
    b = wall(3:4);
    closest = closestPointOnSegment(point(1:2), a, b);
    delta = point(1:2) - closest;
    dist = norm(delta);

    if dist > 1e-9
        normalXY = delta / dist;
    else
        segment = b - a;
        segmentNorm = norm(segment);
        if segmentNorm > 1e-9
            tangent = segment / segmentNorm;
            normalXY = [-tangent(2), tangent(1)];
        else
            normalXY = [1, 0];
        end
    end

    margin = dist - safeClearance;
    normal = [normalXY, 0];
    boundary = [closest + safeClearance * normalXY, point(3)];
end

function rawMargin = signedDistanceToWall(point, wall, bodyClearance)
    a = wall(1:2);
    b = wall(3:4);
    closest = closestPointOnSegment(point(1:2), a, b);
    rawMargin = norm(point(1:2) - closest) - bodyClearance;
end

function closest = closestPointOnSegment(point, a, b)
    d = b - a;
    denom = max(dot(d, d), 1e-12);
    alpha = dot(point - a, d) / denom;
    alpha = max(0, min(1, alpha));
    closest = a + alpha * d;
end

function normal = fallbackObstacleNormal(obs)
    if numel(obs) >= 6
        switch obs(6)
            case 1
                normal = [1, 0, 0];
            case 2
                normal = [0, 0, 1];
            case 3
                normal = [0, 0, -1];
            case 4
                normal = [-1, 0, 0];
            otherwise
                normal = [1, 0, 0];
        end
    else
        normal = [1, 0, 0];
    end
end

function unit = radialUnit(deltaXY, obs)
    radialNorm = norm(deltaXY);
    if radialNorm > 1e-9
        unit = deltaXY / radialNorm;
    else
        normal = fallbackObstacleNormal(obs);
        unit = normal(1:2);
        unitNorm = norm(unit);
        unit = unit / max(unitNorm, 1e-9);
    end
end

function s = verticalSign(dzBelow, dzAbove)
    if dzBelow > dzAbove
        s = -1;
    else
        s = 1;
    end
end
