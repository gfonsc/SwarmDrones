function paths = run_00_setup_branch_paths()
%RUN_00_SETUP_BRANCH_PATHS Configure read-only SwarmDrones paths plus branch paths.

    scriptDir = fileparts(mfilename('fullpath'));
    branchDir = fileparts(scriptDir);
    swarmDir = fileparts(branchDir);

    paths = struct();
    paths.swarmDir = swarmDir;
    paths.branchDir = branchDir;
    paths.scriptsDir = scriptDir;
    paths.functionsDir = fullfile(branchDir, 'functions');
    paths.reportsDir = fullfile(branchDir, 'reports');
    paths.tablesDir = fullfile(branchDir, 'tables');
    paths.resultsDir = fullfile(branchDir, 'results');
    paths.figuresDir = fullfile(branchDir, 'figures');
    paths.baselineFiguresDir = fullfile(paths.figuresDir, 'baseline');
    paths.softFiguresDir = fullfile(paths.figuresDir, 'soft_obstacle');
    paths.iterativeFiguresDir = fullfile(paths.figuresDir, 'iterative_obstacle');
    paths.paperFiguresDir = fullfile(paths.figuresDir, 'paper_ready');
    paths.copiedOriginalsDir = fullfile(branchDir, 'copied_originals');
    paths.generatedDatabaseDir = fullfile(branchDir, 'generated_database');
    paths.deepcDir = fullfile(swarmDir, 'controllers', 'deepc');
    paths.deepcFunctionsDir = fullfile(paths.deepcDir, 'functions');
    paths.databaseDir = fullfile(swarmDir, 'database');
    paths.modelsDir = fullfile(swarmDir, 'models');
    paths.pathPlannerDir = fullfile(swarmDir, 'path_planner');
    paths.pathPlannerFunctionsDir = fullfile(paths.pathPlannerDir, 'functions');

    branchDirs = {
        paths.functionsDir
        paths.reportsDir
        paths.tablesDir
        paths.resultsDir
        paths.figuresDir
        paths.baselineFiguresDir
        paths.softFiguresDir
        paths.iterativeFiguresDir
        paths.paperFiguresDir
        paths.copiedOriginalsDir
    };

    for i = 1:numel(branchDirs)
        if ~exist(branchDirs{i}, 'dir')
            mkdir(branchDirs{i});
        end
    end

    addpath(paths.scriptsDir);
    addpath(paths.functionsDir);
    addpath(paths.deepcDir);
    addpath(paths.deepcFunctionsDir);
    addpath(paths.databaseDir);
    addpath(paths.modelsDir);
    addpath(paths.pathPlannerDir);
    addpath(paths.pathPlannerFunctionsDir);
end
