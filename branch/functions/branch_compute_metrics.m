function metrics = branch_compute_metrics(log, scenario)
%BRANCH_COMPUTE_METRICS Compute branch-side tracking and safety metrics.

    traj = log.pos';
    errVec = log.pos - log.ref;
    err = sqrt(sum(errVec.^2, 1));

    [h, safety] = branch_compute_safety_margin(traj, scenario);

    metrics = struct();
    metrics.qpSuccessRate = mean(log.success);
    metrics.feasibilityRate = metrics.qpSuccessRate;
    metrics.positionRmse = sqrt(mean(err.^2));
    metrics.rmseXYZ = sqrt(mean(errVec.^2, 2));
    metrics.finalError = norm(log.pos(:, end) - scenario.B(:));
    metrics.maxError = max(err);
    metrics.minSafetyMargin = min(h);
    metrics.numSafetyViolations = sum(h < 0);
    if any(h < 0)
        metrics.maxViolationDepth = max(-h(h < 0));
    else
        metrics.maxViolationDepth = 0;
    end

    obstacleClearance = safety.obstacleClearance(isfinite(safety.obstacleClearance));
    if isempty(obstacleClearance)
        metrics.minObstacleClearance = inf;
    else
        metrics.minObstacleClearance = min(obstacleClearance);
    end

    wallClearance = safety.wallClearance(isfinite(safety.wallClearance));
    if isempty(wallClearance)
        metrics.minWallClearance = inf;
    else
        metrics.minWallClearance = min(wallClearance);
    end
    metrics.minCombinedClearance = min(metrics.minObstacleClearance, metrics.minWallClearance);

    metrics.inputEffort = mean(vecnorm(log.u, 2, 1));
    metrics.meanSolverTime = mean(log.solveTime);
    metrics.maxSolverTime = max(log.solveTime);
    metrics.meanSlack = mean(log.slackMean);
    metrics.maxSlack = max(log.slackMax);
    metrics.meanIterations = mean(log.iterationCount);
    metrics.maxIterations = max(log.iterationCount);
    metrics.meanConstraintCount = mean(log.constraintCount);

    metrics.hoverReached = any(traj(:, 3) >= 0.95 * scenario.zHover);
    metrics.usedRightBypass = any(traj(:, 1) > 0.24 & traj(:, 1) < 1.12 & ...
        traj(:, 2) > 2.0 & traj(:, 2) < 4.0);
    metrics.usedAltitudeUp = max(traj(:, 3)) > scenario.zHover + 0.30;
    metrics.usedAltitudeDown = min(traj(traj(:, 2) > 1.0, 3)) < scenario.zHover - 0.25;
    metrics.usedLeftBypass = any(traj(:, 1) < 5.86 & traj(:, 1) > 4.85 & ...
        traj(:, 2) > 2.2 & traj(:, 2) < 4.1);
    metrics.ceilingRespected = all(traj(:, 3) < scenario.zCeiling);
    metrics.floorRespected = all(traj(:, 3) >= -1e-9);
    metrics.collisionFree = metrics.minObstacleClearance > -0.02;
    metrics.wallCollisionFree = metrics.minWallClearance > 0.03;
end
