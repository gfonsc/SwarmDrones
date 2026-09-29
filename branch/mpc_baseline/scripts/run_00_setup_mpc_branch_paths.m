function paths = run_00_setup_mpc_branch_paths()
%RUN_00_SETUP_MPC_BRANCH_PATHS Configure branch-only paths for MPC work.

    scriptDir = fileparts(mfilename('fullpath'));
    mpcDir = fileparts(scriptDir);
    branchDir = fileparts(mpcDir);

    addpath(fullfile(branchDir, 'scripts'));
    addpath(fullfile(branchDir, 'functions'));
    branchPaths = run_00_setup_branch_paths();

    paths = struct();
    paths.rootDir = fileparts(branchPaths.swarmDir);
    paths.swarmDir = branchPaths.swarmDir;
    paths.branchDir = branchDir;
    paths.mpcDir = mpcDir;
    paths.scriptsDir = scriptDir;
    paths.functionsDir = fullfile(mpcDir, 'functions');
    paths.reportsDir = fullfile(mpcDir, 'reports');
    paths.resultsDir = fullfile(mpcDir, 'results');
    paths.tablesDir = fullfile(mpcDir, 'tables');
    paths.figuresDir = fullfile(mpcDir, 'figures');
    paths.paperFiguresDir = fullfile(paths.figuresDir, 'paper_ready');
    paths.comparisonFiguresDir = fullfile(paths.figuresDir, 'comparison');
    paths.scenario00FiguresDir = fullfile(paths.figuresDir, 'scenario_00');
    paths.scenario01FiguresDir = fullfile(paths.figuresDir, 'scenario_01');
    paths.scenario02FiguresDir = fullfile(paths.figuresDir, 'scenario_02');
    paths.copiedDir = fullfile(mpcDir, 'copied_from_tcc');
    paths.deepcResultsDir = fullfile(branchDir, 'results');
    paths.deepcTablesDir = fullfile(branchDir, 'tables');
    paths.articleDraft = fullfile(paths.rootDir, 'GeneralReferences', '2026_CBA_GFonseca.md');
    paths.tccSourceDir = fullfile(paths.rootDir, 'TCC', 'mpc-matlab');
    paths.baselineResultFile = fullfile(paths.deepcResultsDir, 'baseline_results.mat');
    paths.softResultFile = fullfile(paths.deepcResultsDir, 'soft_obstacle_results.mat');
    paths.iterativeResultFile = fullfile(paths.deepcResultsDir, 'iterative_obstacle_results.mat');
    paths.tunedIterativeResultFile = fullfile(paths.deepcResultsDir, 'iterative_obstacle_tuned_results.mat');

    dirs = {
        paths.functionsDir
        paths.reportsDir
        paths.resultsDir
        paths.tablesDir
        paths.figuresDir
        paths.paperFiguresDir
        paths.comparisonFiguresDir
        paths.scenario00FiguresDir
        paths.scenario01FiguresDir
        paths.scenario02FiguresDir
        fullfile(paths.comparisonFiguresDir, 'scenario_00')
        fullfile(paths.comparisonFiguresDir, 'scenario_01')
        fullfile(paths.comparisonFiguresDir, 'scenario_02')
        paths.copiedDir
    };

    for i = 1:numel(dirs)
        if ~exist(dirs{i}, 'dir')
            mkdir(dirs{i});
        end
    end

    addpath(paths.scriptsDir);
    addpath(paths.functionsDir);
    addpath(paths.copiedDir);
end
