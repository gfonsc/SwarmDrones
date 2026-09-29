function scenario = final_apply_obstacle_scenario(common, scenarioId)
%FINAL_APPLY_OBSTACLE_SCENARIO Return the branch scenario for A or B.

    switch upper(char(scenarioId))
        case 'A'
            scenario = common.scenarioA;
        case 'B'
            scenario = common.scenarioB;
        otherwise
            error('final:badScenarioId', 'Scenario must be A or B.');
    end
end
