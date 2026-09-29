function run_01_validate_mpc_source_replay()
%RUN_01_VALIDATE_MPC_SOURCE_REPLAY Validate that the copied TCC MPC source runs on a simple case.

    paths = run_00_setup_mpc_branch_paths();

    phys = crazyflie_params();
    [A_c, B_c, C, D] = linearize_quadrotor(phys);
    sysD = c2d(ss(A_c, B_c, C, D), phys.dt, 'zoh');

    params = struct();
    params.N = 10;
    params.Q_pos = 500 * eye(3);
    params.Q_vel = 1 * eye(3);
    params.Q_ang = 0.5 * eye(3);
    params.R = 0.01 * eye(4);
    params.R_delta = 0.10 * eye(4);
    params.P_term = 10;
    dTmax = phys.T_max - phys.m * phys.g;
    tauMax = max(phys.max_torque);
    params.u_min = [-phys.m * phys.g * 0.5; -tauMax; -tauMax; -tauMax];
    params.u_max = [dTmax; tauMax; tauMax; tauMax];
    params.u_prev = zeros(4, 1);

    x0 = zeros(12, 1);
    x0(3) = 0.2;

    ref = struct();
    ref.pos = repmat([0.2; 0.1; 0.6], 1, params.N);
    ref.vel = zeros(3, params.N);

    [uSeq, xPred, info] = mpc_controller_mosek_v2(sysD.A, sysD.B, C, x0, ref, params);

    validation = struct();
    validation.success = info.success;
    validation.uSeq = uSeq;
    validation.xPred = xPred;
    validation.info = info;
    validation.params = params;
    validation.reference = ref;
    validation.modelA = sysD.A;
    validation.modelB = sysD.B;

    save(fullfile(paths.resultsDir, 'source_replay_validation.mat'), 'validation');

    statusFile = fullfile(paths.reportsDir, 'source_replay_status.md');
    fid = fopen(statusFile, 'w');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '# Source Replay Status\n\n');
    fprintf(fid, '- Source controller: `mpc_controller_mosek_v2.m`\n');
    fprintf(fid, '- Replay success: `%d`\n', info.success);
    fprintf(fid, '- Solver message available: `%d`\n', isfield(info, 'res'));
    fprintf(fid, '- Result file: `results/source_replay_validation.mat`\n');
end
