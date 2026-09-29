function motor = final_generate_motor_efforts_mpc(uCmd, plantCfg, varargin)
%FINAL_GENERATE_MOTOR_EFFORTS_MPC Build a four-channel equivalent effort signal.
%
% The SwarmDrones labyrinth plant uses command inputs
% [vx_cmd; vy_cmd; vz_cmd; yaw_rate_cmd], not physical rotor speeds. This
% function maps those four command demands into four normalized equivalent
% motor-effort channels using one documented linear allocation for both
% DeePC and MPC. The output must not be interpreted as physical rad/s.

    p = inputParser;
    addParameter(p, 'Controller', 'MPC', @ischar);
    parse(p, varargin{:});

    uCmd = coerceControl(uCmd);
    scale = max(abs([plantCfg.uMin(:), plantCfg.uMax(:)]), [], 2);
    scale(scale < 1e-9) = 1;
    uNorm = uCmd ./ scale;

    allocation = [
        -0.25,  0.25, 0.50,  0.25
        -0.25, -0.25, 0.50, -0.25
         0.25, -0.25, 0.50,  0.25
         0.25,  0.25, 0.50, -0.25
    ];
    effort = allocation * uNorm;
    maxAbs = max(1, max(abs(effort), [], 'all'));
    effort = effort / maxAbs;

    motor = struct();
    motor.controller = p.Results.Controller;
    motor.inputRepresentation = '[vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd]';
    motor.effort = effort;
    motor.normalizedCommands = uNorm;
    motor.allocation = allocation;
    motor.labels = {'Motor 1 equiv.', 'Motor 2 equiv.', 'Motor 3 equiv.', 'Motor 4 equiv.'};
    motor.units = 'normalized equivalent effort';
    motor.isPhysicalMotorSpeed = false;
    motor.validForArticle = all(any(abs(effort) > 1e-9, 2));
    motor.notes = ['Equivalent normalized motor effort derived from the four branch command ' ...
        'channels. It is suitable for relative control-demand comparison, not physical motor-speed claims.'];
end

function u = coerceControl(u)
    if isempty(u)
        u = zeros(4, 0);
        return;
    end
    if size(u, 1) == 4
        return;
    end
    if size(u, 2) == 4
        u = u';
        return;
    end
    error('final:badControlShape', 'Control input must be 4 x N or N x 4.');
end
