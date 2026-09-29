clear; clc; close all;

paths = run_00_setup_branch_paths();

fprintf('====================================================\n');
fprintf('  Branch Iterative Obstacle-Aware DeePC Run\n');
fprintf('====================================================\n');

config = branch_extract_demo_config( ...
    'RegenerateIfMissing', true, ...
    'NumSamples', 360, ...
    'Seed', 11, ...
    'TIni', 4, ...
    'TF', 8, ...
    'LambdaY', 700, ...
    'LambdaG', 12, ...
    'PathSpacing', 0.16, ...
    'UseTrainingSetpoints', true, ...
    'SolverName', 'auto', ...
    'SolverVerbose', false);

obstacleOptions = struct();
obstacleOptions.RhoObs = 1e5;
obstacleOptions.SafeDistance = config.scenario.safeDistance;
obstacleOptions.ActivationDistance = 1.00;
obstacleOptions.MaxConstraintsPerStep = 2;
obstacleOptions.IncludeWalls = false;
obstacleOptions.IncludeObstacles = true;
obstacleOptions.MaxIter = 4;
obstacleOptions.Tolerance = 1e-3;

result = branch_run_labyrinth_rollout(config, ...
    'Label', 'branch_iterative_obstacle_deepc', ...
    'ControllerMode', 'iterative', ...
    'MaxSteps', 280, ...
    'ObstacleOptions', obstacleOptions);

figDir = paths.iterativeFiguresDir;
artifacts = branch_plot_labyrinth_results(result, figDir, 'branch_iterative_obstacle_deepc');
result.artifacts = artifacts;
result.obstacleOptions = obstacleOptions;

matFile = fullfile(paths.resultsDir, 'iterative_obstacle_results.mat');
save(matFile, 'result');

fprintf('\nIterative obstacle run complete.\n');
fprintf('  QP success rate: %.1f%%\n', 100 * result.metrics.qpSuccessRate);
fprintf('  Position RMSE:   %.3f m\n', result.metrics.positionRmse);
fprintf('  Final error:     %.3f m\n', result.metrics.finalError);
fprintf('  Min h:           %.3f m\n', result.metrics.minSafetyMargin);
fprintf('  Mean slack:      %.4f\n', result.metrics.meanSlack);
fprintf('  Mean iterations: %.2f\n', result.metrics.meanIterations);
fprintf('  Saved MAT:       %s\n', matFile);
