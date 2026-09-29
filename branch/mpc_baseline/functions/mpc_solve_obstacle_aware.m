function [uSeq, xPred, info] = mpc_solve_obstacle_aware(model, x0, ref, scenario, params, obstacleOptions)
%MPC_SOLVE_OBSTACLE_AWARE Solve one MPC QP with linearized obstacle constraints.

    params = fillDefaults(params, model.defaultParams);
    pNom = coerceNominal(getStructValue(obstacleOptions, 'NominalTrajectory', ref.pos), params.N);
    includeWalls = getStructValue(obstacleOptions, 'IncludeWalls', false);
    includeObstacles = getStructValue(obstacleOptions, 'IncludeObstacles', true);
    activationDistance = getStructValue(obstacleOptions, 'ActivationDistance', 0.55);
    maxConstraintsPerStep = getStructValue(obstacleOptions, 'MaxConstraintsPerStep', 2);
    safeDistance = getStructValue(obstacleOptions, 'SafeDistance', scenario.safeDistance);
    rhoSlack = getStructValue(obstacleOptions, 'RhoSlack', 1e4);

    safeClearance = scenario.droneRadius + safeDistance;
    bodyClearance = scenario.droneRadius;

    [Sx, Su] = buildPredictionMatrices(model.A, model.B, params.N);

    if includeWalls
        walls = labyrinthWallSegments(scenario);
    else
        walls = zeros(0, 4);
    end
    if includeObstacles
        obstacles = scenario.obstacles;
    else
        obstacles = zeros(0, max(5, size(scenario.obstacles, 2)));
    end

    Aobs = zeros(0, size(Su, 2));
    bobs = zeros(0, 1);
    metadata = struct();
    metadata.horizonIndex = zeros(0, 1);
    metadata.sourceType = {};
    metadata.sourceIndex = zeros(0, 1);
    metadata.marginAtNominal = zeros(0, 1);
    metadata.rawClearanceAtNominal = zeros(0, 1);
    metadata.normal = zeros(0, 3);
    metadata.boundaryPoint = zeros(0, 3);

    for j = 1:params.N
        point = pNom(:, j)';
        rows = (j - 1) * size(model.A, 1) + (1:3);
        candidates = {};

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
        order = order(1:min(maxConstraintsPerStep, numel(order)));

        x0Contribution = Sx(rows, :) * x0;
        for idx = order
            lin = candidates{idx};
            normal = lin.normal(:);
            boundary = lin.boundaryPoint(:);

            Aobs(end + 1, :) = -normal' * Su(rows, :); %#ok<AGROW>
            bobs(end + 1, 1) = normal' * x0Contribution - normal' * boundary; %#ok<AGROW>

            metadata.horizonIndex(end + 1, 1) = j; %#ok<AGROW>
            metadata.sourceType{end + 1, 1} = lin.type; %#ok<AGROW>
            metadata.sourceIndex(end + 1, 1) = lin.index; %#ok<AGROW>
            metadata.marginAtNominal(end + 1, 1) = lin.margin; %#ok<AGROW>
            metadata.rawClearanceAtNominal(end + 1, 1) = lin.rawClearance; %#ok<AGROW>
            metadata.normal(end + 1, :) = normal'; %#ok<AGROW>
            metadata.boundaryPoint(end + 1, :) = boundary'; %#ok<AGROW>
        end
    end

    extra = struct();
    extra.A = Aobs;
    extra.b = bobs;
    extra.useSlack = ~isempty(Aobs);
    extra.rhoSlack = rhoSlack;
    extra.metadata = metadata;

    [uSeq, xPred, info] = mpc_solve_tracking(model, x0, ref, params, 'ExtraConstraints', extra);
end

function params = fillDefaults(params, defaults)
    fields = fieldnames(defaults);
    for i = 1:numel(fields)
        if ~isfield(params, fields{i}) || isempty(params.(fields{i}))
            params.(fields{i}) = defaults.(fields{i});
        end
    end
end

function value = getStructValue(s, fieldName, defaultValue)
    if isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end

function pNom = coerceNominal(pNom, N)
    if isempty(pNom)
        pNom = zeros(3, N);
        return;
    end
    if size(pNom, 1) ~= 3
        pNom = pNom';
    end
    if size(pNom, 2) < N
        pNom = [pNom, repmat(pNom(:, end), 1, N - size(pNom, 2))];
    else
        pNom = pNom(:, 1:N);
    end
end

function [Sx, Su] = buildPredictionMatrices(A, B, N)
    n = size(A, 1);
    m = size(B, 2);
    Sx = zeros(n * N, n);
    Su = zeros(n * N, m * N);
    A_pow = eye(n);
    for k = 1:N
        A_pow = A * A_pow;
        rows = (k - 1) * n + (1:n);
        Sx(rows, :) = A_pow;
        for j = 1:k
            cols = (j - 1) * m + (1:m);
            Su(rows, cols) = A^(k - j) * B;
        end
    end
end
