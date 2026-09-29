function validation = final_validate_motor_efforts(deepcResult, mpcResult)
%FINAL_VALIDATE_MOTOR_EFFORTS Validate four-channel effort comparability.

    validation = struct();
    validation.deepcOutputsPhysicalMotors = false;
    validation.mpcOutputsPhysicalMotors = false;
    validation.deepcInputRepresentation = '[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]';
    validation.mpcInputRepresentation = '[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]';
    validation.mappingType = 'equivalent normalized command effort';
    validation.sameMappingForBothControllers = true;
    validation.allMpcChannelsNonPlaceholder = all(any(abs(mpcResult.motorEffort.effort) > 1e-9, 2));
    validation.allDeepcChannelsNonPlaceholder = all(any(abs(deepcResult.motorEffort.effort) > 1e-9, 2));
    validation.validForArticle = validation.allMpcChannelsNonPlaceholder && ...
        validation.allDeepcChannelsNonPlaceholder && ...
        strcmp(mpcResult.motorEffort.units, deepcResult.motorEffort.units);
    validation.warning = ['These are not measured or simulated rotor speeds. They are normalized ' ...
        'equivalent command-effort channels created with the same allocation for DeePC and MPC.'];
end
