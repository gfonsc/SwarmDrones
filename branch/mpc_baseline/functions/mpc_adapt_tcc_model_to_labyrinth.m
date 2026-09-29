function model = mpc_adapt_tcc_model_to_labyrinth(scenarioData, varargin)
%MPC_ADAPT_TCC_MODEL_TO_LABYRINTH Adapt the TCC condensed-QP structure to the labyrinth plant.

    p = inputParser;
    addParameter(p, 'UseSourceBestWeights', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    dt = scenarioData.plantCfg.dt;
    tauVel = scenarioData.plantCfg.tauVel;
    tauYaw = scenarioData.plantCfg.tauYaw;

    alphaVel = min(1, dt / max(tauVel, dt));
    alphaYaw = min(1, dt / max(tauYaw, dt));

    A = eye(8);
    B = zeros(8, 4);

    A(1:3, 4:6) = dt * (1 - alphaVel) * eye(3);
    A(4:6, 4:6) = (1 - alphaVel) * eye(3);
    B(1:3, 1:3) = dt * alphaVel * eye(3);
    B(4:6, 1:3) = alphaVel * eye(3);

    A(7, 8) = dt * (1 - alphaYaw);
    A(8, 8) = (1 - alphaYaw);
    B(7, 4) = dt * alphaYaw;
    B(8, 4) = alphaYaw;

    Cpos = [eye(3), zeros(3, 5)];
    Cvel = [zeros(3), eye(3), zeros(3, 2)];
    Cyaw = [zeros(1, 6), 1, 0];
    CyawRate = [zeros(1, 7), 1];

    if opts.UseSourceBestWeights
        qPosScale = 500;
        qVelScale = 1;
        rScale = 0.01;
        rDeltaScale = 0.10;
        pTermScale = 10;
    else
        qPosScale = 120;
        qVelScale = 5;
        rScale = 0.05;
        rDeltaScale = 0.10;
        pTermScale = 5;
    end

    defaultParams = struct();
    defaultParams.N = 8;
    defaultParams.Q_pos = qPosScale * eye(3);
    defaultParams.Q_vel = qVelScale * eye(3);
    defaultParams.Q_yaw = 0.5;
    defaultParams.Q_yaw_rate = 0.05;
    defaultParams.R = rScale * eye(4);
    defaultParams.R_delta = rDeltaScale * eye(4);
    defaultParams.P_term = pTermScale;
    defaultParams.u_min = scenarioData.plantCfg.uMin(:);
    defaultParams.u_max = scenarioData.plantCfg.uMax(:);
    defaultParams.u_prev = zeros(4, 1);
    defaultParams.solverOpts = struct('solver', 'auto', 'verbose', false);

    model = struct();
    model.A = A;
    model.B = B;
    model.Cpos = Cpos;
    model.Cvel = Cvel;
    model.Cyaw = Cyaw;
    model.CyawRate = CyawRate;
    model.dt = dt;
    model.tauVel = tauVel;
    model.tauYaw = tauYaw;
    model.alphaVel = alphaVel;
    model.alphaYaw = alphaYaw;
    model.stateNames = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'yaw', 'yaw_rate'};
    model.inputNames = scenarioData.inputNames;
    model.defaultParams = defaultParams;
    model.sourceTemplate = 'mpc_controller_mosek_v2';
    model.notes = ['Branch adaptation of the TCC condensed-QP MPC structure to the ' ...
        'SwarmDrones labyrinth kinematic plant.'];
end
