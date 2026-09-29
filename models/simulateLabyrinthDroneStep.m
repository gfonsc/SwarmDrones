function [stateNext, y, uClipped] = simulateLabyrinthDroneStep(state, u, cfg)
% SIMULATELABYRINTHDRONESTEP Kinematic drone model used for labyrinth DeePC data.
%
%   State: [x y z vx vy vz yaw yawRate]'
%   Input: [vx_cmd vy_cmd vz_cmd yawRate_cmd]'
%   Output y uses the same ordering as the state.

    uClipped = max(cfg.uMin(:), min(cfg.uMax(:), u(:)));

    state = state(:);
    pos = state(1:3);
    vel = state(4:6);
    yaw = state(7);
    yawRate = state(8);

    alphaVel = min(1, cfg.dt / max(cfg.tauVel, cfg.dt));
    alphaYaw = min(1, cfg.dt / max(cfg.tauYaw, cfg.dt));

    vel = vel + alphaVel * (uClipped(1:3) - vel);
    yawRate = yawRate + alphaYaw * (uClipped(4) - yawRate);

    pos = pos + cfg.dt * vel;
    yaw = wrapToPiLocal(yaw + cfg.dt * yawRate);

    if isfield(cfg, 'stateMin') && isfield(cfg, 'stateMax')
        posMin = cfg.stateMin(1:3);
        posMax = cfg.stateMax(1:3);
        posBeforeClamp = pos;
        pos = max(posMin, min(posMax, pos));
        for i = 1:3
            if pos(i) ~= posBeforeClamp(i)
                vel(i) = 0;
            end
        end
    end

    stateNext = [pos; vel; yaw; yawRate];
    y = stateNext;
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
