function paths = run_00_setup_final_comparison_paths()
%RUN_00_SETUP_FINAL_COMPARISON_PATHS Configure paths for final article comparison.

    scriptDir = fileparts(mfilename('fullpath'));
    finalDir = fileparts(scriptDir);
    branchDir = fileparts(finalDir);
    swarmDir = fileparts(branchDir);
    rootDir = fileparts(swarmDir);

    paths = struct();
    paths.rootDir = rootDir;
    paths.swarmDir = swarmDir;
    paths.branchDir = branchDir;
    paths.finalDir = finalDir;
    paths.scriptsDir = scriptDir;
    paths.functionsDir = fullfile(finalDir, 'functions');
    paths.reportsDir = fullfile(finalDir, 'reports');
    paths.resultsDir = fullfile(finalDir, 'results');
    paths.tablesDir = fullfile(finalDir, 'tables');
    paths.figuresDir = fullfile(finalDir, 'figures');
    paths.diagnosticsDir = fullfile(paths.figuresDir, 'diagnostics');
    paths.paperReadyDir = fullfile(paths.figuresDir, 'paper_ready');
    paths.branchScriptsDir = fullfile(branchDir, 'scripts');
    paths.branchFunctionsDir = fullfile(branchDir, 'functions');
    paths.mpcScriptsDir = fullfile(branchDir, 'mpc_baseline', 'scripts');
    paths.mpcFunctionsDir = fullfile(branchDir, 'mpc_baseline', 'functions');
    paths.mpcCopiedDir = fullfile(branchDir, 'mpc_baseline', 'copied_from_tcc');

    dirs = {
        paths.finalDir
        paths.scriptsDir
        paths.functionsDir
        paths.reportsDir
        paths.resultsDir
        paths.tablesDir
        paths.figuresDir
        paths.diagnosticsDir
        paths.paperReadyDir
    };

    for i = 1:numel(dirs)
        if ~exist(dirs{i}, 'dir')
            mkdir(dirs{i});
        end
    end

    addpath(paths.branchScriptsDir);
    addpath(paths.branchFunctionsDir);
    if exist('run_00_setup_branch_paths', 'file') == 2
        run_00_setup_branch_paths();
    end

    addpath(paths.mpcScriptsDir);
    addpath(paths.mpcFunctionsDir);
    addpath(paths.mpcCopiedDir);
    addpath(paths.scriptsDir);
    addpath(paths.functionsDir);
end
