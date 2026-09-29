function model = deepcSetupSegments(uSegments, ySegments, params)
% DEEPCSETUPSEGMENTS Build a DeePC model from multiple independent runs.
%
%   model = deepcSetupSegments(uSegments, ySegments, params)
%
%   Each cell in uSegments/ySegments is one continuous experiment. Hankel
%   columns are built inside each experiment and then concatenated, avoiding
%   artificial columns that would cross discontinuities between CSV files.

    funcPath = fullfile(fileparts(mfilename('fullpath')), 'functions');
    addpath(funcPath);

    assert(iscell(uSegments) && iscell(ySegments), ...
        'uSegments and ySegments must be cell arrays.');
    assert(numel(uSegments) == numel(ySegments), ...
        'uSegments and ySegments must have the same number of cells.');

    valid = false(1, numel(uSegments));
    for i = 1:numel(uSegments)
        valid(i) = ~isempty(uSegments{i}) && ~isempty(ySegments{i});
    end
    uSegments = uSegments(valid);
    ySegments = ySegments(valid);
    assert(~isempty(uSegments), 'No valid data segments were supplied.');

    [m, ~] = size(uSegments{1});
    [p, ~] = size(ySegments{1});
    for i = 1:numel(uSegments)
        assert(size(uSegments{i}, 1) == m, 'Input dimension mismatch in segment %d.', i);
        assert(size(ySegments{i}, 1) == p, 'Output dimension mismatch in segment %d.', i);
        assert(size(uSegments{i}, 2) == size(ySegments{i}, 2), ...
            'Input/output sample mismatch in segment %d.', i);
    end

    totalSamples = sum(cellfun(@(u) size(u, 2), uSegments));

    fprintf('====================================================\n');
    fprintf('  DeePC Offline Setup from Segmented Data\n');
    fprintf('====================================================\n');
    fprintf('  Segments:     %d\n', numel(uSegments));
    fprintf('  Inputs (m):   %d\n', m);
    fprintf('  Outputs (p):  %d\n', p);
    fprintf('  Total samples:%d\n', totalSamples);

    fprintf('\n--- Step 1: Validating parameters ---\n');
    params = validateParams(params, m, p);
    T_ini = params.T_ini;
    T_f = params.T_f;
    L = T_ini + T_f;

    fprintf('  T_ini = %d (initial trajectory length)\n', T_ini);
    fprintf('  T_f   = %d (prediction horizon)\n', T_f);
    fprintf('  lambda_y = %.2e (initial condition penalty)\n', params.lambda_y);
    fprintf('  lambda_g = %.1f (g regularization, qNorm=%d)\n', params.lambda_g, params.qNorm);

    fprintf('\n--- Step 2: Building segmented Hankel matrices ---\n');
    Up = [];
    Yp = [];
    Uf = [];
    Yf = [];
    keptSegments = 0;
    keptSamples = 0;
    for i = 1:numel(uSegments)
        Ti = size(uSegments{i}, 2);
        if Ti < L
            warning('deepc:shortSegment', ...
                'Skipping IDSIA segment %d because it has %d samples and needs at least %d.', ...
                i, Ti, L);
            continue;
        end

        [Upi, Ypi, Ufi, Yfi] = buildHankelMatrix(uSegments{i}, ySegments{i}, T_ini, T_f);
        Up = [Up, Upi]; %#ok<AGROW>
        Yp = [Yp, Ypi]; %#ok<AGROW>
        Uf = [Uf, Ufi]; %#ok<AGROW>
        Yf = [Yf, Yfi]; %#ok<AGROW>
        keptSegments = keptSegments + 1;
        keptSamples = keptSamples + Ti;
    end

    nG = size(Up, 2);
    fprintf('  Kept segments: %d / %d\n', keptSegments, numel(uSegments));
    fprintf('  Up: %d x %d (past inputs)\n', size(Up, 1), size(Up, 2));
    fprintf('  Yp: %d x %d (past outputs)\n', size(Yp, 1), size(Yp, 2));
    fprintf('  Uf: %d x %d (future inputs)\n', size(Uf, 1), size(Uf, 2));
    fprintf('  Yf: %d x %d (future outputs)\n', size(Yf, 1), size(Yf, 2));
    fprintf('  Decision variable g: %d x 1\n', nG);

    fprintf('\n--- Step 3: Persistency of excitation check ---\n');
    uDataForCheck = zeros(m, keptSamples);
    cursor = 1;
    for i = 1:numel(uSegments)
        Ti = size(uSegments{i}, 2);
        if Ti < L
            continue;
        end
        uDataForCheck(:, cursor:cursor + Ti - 1) = uSegments{i};
        cursor = cursor + Ti;
    end
    checkPersistency(Up, Yp, Uf, uDataForCheck, params);

    fprintf('\n--- Step 4: Computing g_r ---\n');
    gRef = computeSteadyStateG(Up, Yp, Uf, Yf, params);

    fprintf('\n--- Step 5: Building Hessian ---\n');
    H = buildHessian(Uf, Yf, Yp, nG, params);
    condH = cond(H);
    fprintf('  Hessian size: %d x %d\n', size(H, 1), size(H, 2));
    fprintf('  Condition number: %.2e\n', condH);
    if condH > 1e12
        warning('deepc:illConditioned', ...
            'Hessian condition number %.2e is very large. Consider increasing epsReg or lambda_g.', condH);
    end

    fprintf('\n--- Step 6: Building constraints ---\n');
    [Aeq, Aineq, bIneq] = buildConstraintMatrix(Up, Uf, Yf, params);
    fprintf('  Equality constraints:   %d (U_p g = u_ini)\n', size(Aeq, 1));
    fprintf('  Inequality constraints: %d (u/y bounds)\n', size(Aineq, 1));

    fprintf('\n--- Step 7: Precomputing products for online phase ---\n');
    precomp = struct();
    precomp.neg2_YfQ = -2 * Yf' * params.Q_full;
    precomp.neg2_lambdaY_YpT = -2 * params.lambda_y * Yp';
    precomp.fConst = -2 * (Uf' * params.R_full * params.u_r_full ...
                         + params.lambda_g * gRef);
    fprintf('  Precomputed: neg2_YfQ (%d x %d)\n', size(precomp.neg2_YfQ));
    fprintf('  Precomputed: neg2_lambdaY_YpT (%d x %d)\n', size(precomp.neg2_lambdaY_YpT));
    fprintf('  Precomputed: fConst (%d x 1)\n', length(precomp.fConst));

    model = struct();
    model.Up = Up;
    model.Yp = Yp;
    model.Uf = Uf;
    model.Yf = Yf;
    model.H = H;
    model.Aeq = Aeq;
    model.Aineq = Aineq;
    model.bIneq = bIneq;
    model.precomp = precomp;
    model.gRef = gRef;
    model.params = params;
    model.dims = struct('m', m, 'p', p, 'T_ini', T_ini, 'T_f', T_f, ...
        'nG', nG, 'T', keptSamples, 'segments', keptSegments);

    fprintf('\n====================================================\n');
    fprintf('  Segmented DeePC Setup Complete\n');
    fprintf('  Ready for online control with deepcAlgorithm()\n');
    fprintf('====================================================\n');
end
