function [A_obs, b_obs, metadata] = branch_build_obstacle_constraints(config, pNom, obstacleOptions)
%BRANCH_BUILD_OBSTACLE_CONSTRAINTS Build linearized obstacle constraints over the horizon.

    pNom = coerceNominal(pNom, config.T_f);
    safeDistance = getOption(obstacleOptions, 'SafeDistance', config.scenario.safeDistance);
    activationDistance = getOption(obstacleOptions, 'ActivationDistance', 0.55);
    maxConstraintsPerStep = getOption(obstacleOptions, 'MaxConstraintsPerStep', 2);
    includeWalls = getOption(obstacleOptions, 'IncludeWalls', true);
    includeObstacles = getOption(obstacleOptions, 'IncludeObstacles', true);

    safeClearance = config.scenario.droneRadius + safeDistance;
    bodyClearance = config.scenario.droneRadius;

    if includeWalls
        walls = labyrinthWallSegments(config.scenario);
    else
        walls = zeros(0, 4);
    end

    if includeObstacles
        obstacles = config.scenario.obstacles;
    else
        obstacles = zeros(0, max(5, size(config.scenario.obstacles, 2)));
    end

    A_obs = zeros(0, config.model.dims.nG);
    b_obs = zeros(0, 1);

    metadata = struct();
    metadata.horizonIndex = zeros(0, 1);
    metadata.sourceType = {};
    metadata.sourceIndex = zeros(0, 1);
    metadata.marginAtNominal = zeros(0, 1);
    metadata.rawClearanceAtNominal = zeros(0, 1);
    metadata.normal = zeros(0, 3);
    metadata.boundaryPoint = zeros(0, 3);

    for j = 1:config.T_f
        candidates = {};
        point = pNom(:, j)';

        for i = 1:size(obstacles, 1)
            feature = struct('type', 'obstacle', 'index', i, 'data', obstacles(i, :));
            lin = branch_linearize_obstacle_constraints(feature, point, ...
                'SafeClearance', safeClearance, 'BodyClearance', bodyClearance);
            if lin.margin <= activationDistance
                candidates{end + 1} = lin; %#ok<AGROW>
            end
        end

        for i = 1:size(walls, 1)
            feature = struct('type', 'wall', 'index', i, 'data', walls(i, :));
            lin = branch_linearize_obstacle_constraints(feature, point, ...
                'SafeClearance', safeClearance, 'BodyClearance', bodyClearance);
            if lin.margin <= activationDistance
                candidates{end + 1} = lin; %#ok<AGROW>
            end
        end

        if isempty(candidates)
            continue;
        end

        margins = cellfun(@(c) c.margin, candidates);
        [~, order] = sort(margins, 'ascend');
        takeCount = min(maxConstraintsPerStep, numel(order));
        order = order(1:takeCount);

        Pj = config.positionMap{j};
        offset = config.positionOffset(:);
        for idx = order
            lin = candidates{idx};
            normal = lin.normal(:);
            boundary = lin.boundaryPoint(:);
            A_obs(end + 1, :) = -normal' * Pj; %#ok<AGROW>
            b_obs(end + 1, 1) = normal' * offset - normal' * boundary; %#ok<AGROW>

            metadata.horizonIndex(end + 1, 1) = j; %#ok<AGROW>
            metadata.sourceType{end + 1, 1} = lin.type; %#ok<AGROW>
            metadata.sourceIndex(end + 1, 1) = lin.index; %#ok<AGROW>
            metadata.marginAtNominal(end + 1, 1) = lin.margin; %#ok<AGROW>
            metadata.rawClearanceAtNominal(end + 1, 1) = lin.rawClearance; %#ok<AGROW>
            metadata.normal(end + 1, :) = normal'; %#ok<AGROW>
            metadata.boundaryPoint(end + 1, :) = boundary'; %#ok<AGROW>
        end
    end
end

function pNom = coerceNominal(pNom, T_f)
    if size(pNom, 1) == 3 && size(pNom, 2) == T_f
        return;
    end
    if size(pNom, 2) == 3 && size(pNom, 1) == T_f
        pNom = pNom';
        return;
    end
    error('branch:badNominalHorizon', 'Nominal horizon must be 3 x T_f or T_f x 3.');
end

function value = getOption(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
