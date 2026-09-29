function scenarioData = mpc_load_branch_scenario(varargin)
%MPC_LOAD_BRANCH_SCENARIO Load the existing branch labyrinth scenario and path.

    p = inputParser;
    addParameter(p, 'UseExistingBaseline', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    paths = run_00_setup_mpc_branch_paths();

    baselineResult = [];
    if opts.UseExistingBaseline && isfile(paths.baselineResultFile)
        S = load(paths.baselineResultFile, 'result');
        baselineResult = S.result;
    end

    if ~isempty(baselineResult)
        scenario = baselineResult.scenario;
        path = baselineResult.path;
        maxSteps = numel(baselineResult.log.time);
        pathSource = baselineResult.pathSource;
        initialState = [scenario.A(:); 0; 0; 0; pi / 2; 0];
        plantCfg = scenario.model;
        dataOrigin = 'existing_branch_baseline_result';
    else
        evalc('cfg = branch_extract_demo_config(''SolverVerbose'', false);');
        scenario = cfg.scenario;
        path = cfg.path;
        maxSteps = 240;
        pathSource = cfg.pathSource;
        initialState = cfg.initialState;
        plantCfg = cfg.plantCfg;
        dataOrigin = 'branch_extract_demo_config';
    end

    scenarioData = struct();
    scenarioData.paths = paths;
    scenarioData.dataOrigin = dataOrigin;
    scenarioData.scenario = scenario;
    scenarioData.path = path;
    scenarioData.pathSource = pathSource;
    scenarioData.dt = scenario.dt;
    scenarioData.initialState = initialState;
    scenarioData.plantCfg = plantCfg;
    scenarioData.maxSteps = maxSteps;
    scenarioData.baselineResult = baselineResult;
    scenarioData.inputNames = scenario.inputNames;
    scenarioData.outputNames = scenario.outputNames;
    scenarioData.safeClearance = scenario.droneRadius + scenario.safeDistance;
end
