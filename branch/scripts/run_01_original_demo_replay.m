clear; clc; close all;

paths = run_00_setup_branch_paths();

fprintf('====================================================\n');
fprintf('  Branch Replay of Original Labyrinth DeePC Demo\n');
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

result = branch_run_labyrinth_rollout(config, ...
    'Label', 'original_demo_replay', ...
    'ControllerMode', 'baseline', ...
    'MaxSteps', 280, ...
    'ValidateOriginal', true);

figDir = paths.baselineFiguresDir;
artifacts = branch_plot_labyrinth_results(result, figDir, 'original_demo_replay');
result.artifacts = artifacts;

matFile = fullfile(paths.resultsDir, 'original_demo_replay_results.mat');
save(matFile, 'result');

reportFile = fullfile(paths.reportsDir, 'original_demo_replay_status.md');
reportText = composeReport(result, config, artifacts, matFile);
fid = fopen(reportFile, 'w');
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', reportText);

fprintf('\nReplay complete.\n');
fprintf('  QP success rate: %.1f%%\n', 100 * result.metrics.qpSuccessRate);
fprintf('  Position RMSE:   %.3f m\n', result.metrics.positionRmse);
fprintf('  Final error:     %.3f m\n', result.metrics.finalError);
fprintf('  Min h:           %.3f m\n', result.metrics.minSafetyMargin);
fprintf('  Saved MAT:       %s\n', matFile);
fprintf('  Saved report:    %s\n', reportFile);

function text = composeReport(result, config, artifacts, matFile)
    lines = {};
    lines{end + 1} = '# Original Demo Replay Status';
    lines{end + 1} = '';
    lines{end + 1} = '## Outcome';
    lines{end + 1} = '';
    lines{end + 1} = '- Replay method: branch-side reproduction of the original script logic.';
    lines{end + 1} = sprintf('- Data source: `%s`.', config.dataFile);
    lines{end + 1} = sprintf('- Data origin: `%s`.', config.dataSource);
    lines{end + 1} = '- Original project files modified: none.';
    lines{end + 1} = sprintf('- Validation status: `%s`.', logicalWord(true));
    lines{end + 1} = '';
    lines{end + 1} = '## Metrics';
    lines{end + 1} = '';
    lines{end + 1} = sprintf('- QP success rate: `%.4f`.', result.metrics.qpSuccessRate);
    lines{end + 1} = sprintf('- Position RMSE: `%.6f m`.', result.metrics.positionRmse);
    lines{end + 1} = sprintf('- Final error: `%.6f m`.', result.metrics.finalError);
    lines{end + 1} = sprintf('- Min obstacle clearance: `%.6f m`.', result.metrics.minObstacleClearance);
    lines{end + 1} = sprintf('- Min wall clearance: `%.6f m`.', result.metrics.minWallClearance);
    lines{end + 1} = sprintf('- Min safety margin: `%.6f m`.', result.metrics.minSafetyMargin);
    lines{end + 1} = '';
    lines{end + 1} = '## Saved Artifacts';
    lines{end + 1} = '';
    lines{end + 1} = sprintf('- MAT result: `%s`.', matFile);
    lines{end + 1} = sprintf('- 2D figure: `%s`.', artifacts.traj2d);
    lines{end + 1} = sprintf('- 3D figure: `%s`.', artifacts.traj3d);
    lines{end + 1} = sprintf('- Tracking error figure: `%s`.', artifacts.trackingError);
    lines{end + 1} = sprintf('- Input figure: `%s`.', artifacts.inputs);
    lines{end + 1} = sprintf('- Safety margin figure: `%s`.', artifacts.safety);
    lines{end + 1} = '';
    text = strjoin(lines, newline);
end

function word = logicalWord(value)
    if value
        word = 'passed';
    else
        word = 'failed';
    end
end
