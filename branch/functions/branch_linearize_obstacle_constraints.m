function linearization = branch_linearize_obstacle_constraints(feature, pNom, varargin)
%BRANCH_LINEARIZE_OBSTACLE_CONSTRAINTS Linearize one wall or obstacle safety constraint.

    p = inputParser;
    addParameter(p, 'SafeClearance', 0.25, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'BodyClearance', 0.08, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parse(p, varargin{:});
    opts = p.Results;

    pNom = pNom(:)';
    if numel(pNom) ~= 3
        error('branch:badNominalPoint', 'Nominal point must be a 3D position.');
    end

    switch lower(feature.type)
        case 'obstacle'
            [margin, normal, boundaryPoint] = signedDistanceToInflatedCylinder( ...
                pNom, feature.data, opts.SafeClearance);
            rawClearance = signedDistanceToPhysicalCylinder(pNom, feature.data, opts.BodyClearance);

        case 'wall'
            [margin, normal, boundaryPoint] = signedDistanceToInflatedWall( ...
                pNom, feature.data, opts.SafeClearance);
            rawClearance = signedDistanceToWall(pNom, feature.data, opts.BodyClearance);

        otherwise
            error('branch:badFeatureType', 'Unsupported feature type: %s', feature.type);
    end

    linearization = struct();
    linearization.type = feature.type;
    linearization.index = feature.index;
    linearization.margin = margin;
    linearization.rawClearance = rawClearance;
    linearization.normal = normal(:);
    linearization.boundaryPoint = boundaryPoint(:);
end

function [margin, normal, boundary] = signedDistanceToInflatedCylinder(point, obs, safeClearance)
    zMin = obs(3);
    zMax = obs(4);
    inflatedRadius = obs(5) + safeClearance;

    centerXY = obs(1:2);
    deltaXY = point(1:2) - centerXY;
    radialDist = norm(deltaXY);
    radialMargin = radialDist - inflatedRadius;
    dzBelow = zMin - point(3);
    dzAbove = point(3) - zMax;
    verticalGap = max([dzBelow, dzAbove, 0]);
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
        boundary = point - margin * normal;
        return;
    end

    if verticalGap == 0
        margin = radialMargin;
        if radialDist > 1e-9
            normal = [deltaXY / radialDist, 0];
        else
            normal = fallbackObstacleNormal(obs);
        end
        boundary = point - margin * normal;
        return;
    end

    if radialMargin <= 0
        margin = verticalGap;
        normal = [0, 0, verticalSign(dzBelow, dzAbove)];
        boundary = point - margin * normal;
        return;
    end

    radialDir = radialUnit(deltaXY, obs);
    cornerVector = [radialMargin * radialDir, verticalSign(dzBelow, dzAbove) * verticalGap];
    cornerNorm = norm(cornerVector);
    if cornerNorm > 1e-9
        normal = cornerVector / cornerNorm;
    else
        normal = fallbackObstacleNormal(obs);
    end
    margin = hypot(radialMargin, verticalGap);
    boundary = point - margin * normal;
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
    alpha = dot(point - a, d) / max(dot(d, d), 1e-12);
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
        unit = unit / max(norm(unit), 1e-9);
    end
end

function s = verticalSign(dzBelow, dzAbove)
    if dzBelow > dzAbove
        s = -1;
    else
        s = 1;
    end
end
