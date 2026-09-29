function mapped = mpc_motor_mapping(u, varargin)
%MPC_MOTOR_MAPPING Convert 4-channel inputs into a comparable four-effort representation.

    p = inputParser;
    addParameter(p, 'Mode', 'direct_command', @ischar);
    addParameter(p, 'Params', struct(), @isstruct);
    parse(p, varargin{:});
    opts = p.Results;

    u = coerceInput(u);

    switch lower(opts.Mode)
        case 'direct_command'
            mapped = struct();
            mapped.effort = u;
            mapped.labels = {'vx\_cmd', 'vy\_cmd', 'vz\_cmd', 'yaw\_rate\_cmd'};
            mapped.units = {'m/s', 'm/s', 'm/s', 'rad/s'};
            mapped.mode = 'direct_command';
            mapped.notes = ['The labyrinth branch plant is command-based. The four-effort ' ...
                'comparison uses the native command channels rather than physical motor speeds.'];

        case 'thrust_torque'
            params = fillDefaultMixerParams(opts.Params);
            M = [
                params.Kt, params.Kt, params.Kt, params.Kt;
                0, params.armLength * params.Kt, 0, -params.armLength * params.Kt;
                -params.armLength * params.Kt, 0, params.armLength * params.Kt, 0;
                params.Kc, -params.Kc, params.Kc, -params.Kc
            ];
            omegaSq = M \ u;
            omegaSq = max(omegaSq, 0);

            mapped = struct();
            mapped.effort = sqrt(omegaSq);
            mapped.motorSquared = omegaSq;
            mapped.labels = {'motor_1', 'motor_2', 'motor_3', 'motor_4'};
            mapped.units = {'rad/s', 'rad/s', 'rad/s', 'rad/s'};
            mapped.mode = 'thrust_torque';
            mapped.notes = 'Approximate mixer inversion using the copied Crazyflie parameter convention.';

        otherwise
            error('mpc:badMotorMappingMode', 'Unsupported motor mapping mode: %s', opts.Mode);
    end
end

function u = coerceInput(u)
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
    error('mpc:badInputShape', 'Expected 4 x N or N x 4 input matrix.');
end

function params = fillDefaultMixerParams(params)
    if ~isfield(params, 'Kt'), params.Kt = 2.23e-8; end
    if ~isfield(params, 'Kc'), params.Kc = 4.80e-10; end
    if ~isfield(params, 'armLength'), params.armLength = 0.0353; end
end
