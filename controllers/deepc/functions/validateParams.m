function params = validateParams(params, m, p)
% VALIDATEPARAMS  Validate and complete DeePC hyperparameters.
%
%   params = validateParams(params, m, p)
%
%   Checks required fields, sets defaults for optional ones, and expands
%   per-step weight matrices to full-horizon block-diagonal form.
%
%   Required fields:
%       T_ini   - Initial trajectory length for state estimation
%       T_f     - Prediction horizon
%
%   Optional fields (with defaults):
%       Q          - Output tracking weight. (p x p) per-step or (p*T_f x p*T_f)
%                    Default: eye(p)
%       R          - Input penalty weight. (m x m) per-step or (m*T_f x m*T_f)
%                    Default: 0.01 * eye(m)
%       lambda_y   - Penalty on ||Y_p g - y_ini||^2. Default: 1e8
%                    Thesis Section 5.3.2: "choose as large as possible"
%       lambda_g   - Regularization weight on g. Default: 500
%                    Thesis Section 5.3.2: "lambda_g = 500 performs well"
%       qNorm      - Regularization norm (1 or 2). Default: 2
%                    Thesis: q=2 recommended for real-time
%       u_r        - Steady-state input reference (m x 1). Default: zeros(m,1)
%       y_r        - Steady-state output reference (p x 1). Default: zeros(p,1)
%       u_min      - Input lower bounds (m x 1). Default: -Inf
%       u_max      - Input upper bounds (m x 1). Default: +Inf
%       y_min      - Output lower bounds (p x 1). Default: -Inf
%       y_max      - Output upper bounds (p x 1). Default: +Inf
%       epsReg     - Numerical regularization added to Hessian diagonal.
%                    Default: 1e-8
%       verbose    - Print diagnostic info. Default: true
%
%   Reference: Coulson thesis, Section 5.3.2 — Hyperparameter guidelines

    %% Required fields
    assert(isfield(params, 'T_ini'), 'params.T_ini is required.');
    assert(isfield(params, 'T_f'),   'params.T_f is required.');

    T_ini = params.T_ini;
    T_f   = params.T_f;

    assert(T_ini >= 1, 'T_ini must be >= 1.');
    assert(T_f   >= 1, 'T_f must be >= 1.');

    %% Defaults — based on thesis Section 5.3.2 and Table 5.1
    if ~isfield(params, 'Q'),        params.Q = eye(p);                  end
    if ~isfield(params, 'R'),        params.R = 0.01 * eye(m);           end
    if ~isfield(params, 'lambda_y'), params.lambda_y = 1e8;              end
    if ~isfield(params, 'lambda_g'), params.lambda_g = 500;              end
    if ~isfield(params, 'qNorm'),    params.qNorm = 2;                   end
    if ~isfield(params, 'u_r'),      params.u_r = zeros(m, 1);           end
    if ~isfield(params, 'y_r'),      params.y_r = zeros(p, 1);           end
    if ~isfield(params, 'u_min'),    params.u_min = -inf(m, 1);          end
    if ~isfield(params, 'u_max'),    params.u_max =  inf(m, 1);          end
    if ~isfield(params, 'y_min'),    params.y_min = -inf(p, 1);          end
    if ~isfield(params, 'y_max'),    params.y_max =  inf(p, 1);          end
    if ~isfield(params, 'epsReg'),   params.epsReg = 1e-8;               end
    if ~isfield(params, 'verbose'),  params.verbose = true;              end

    %% Validate norm choice
    assert(params.qNorm == 1 || params.qNorm == 2, ...
        'qNorm must be 1 or 2. Thesis recommends qNorm = 2.');

    if params.qNorm == 1
        warning('deepc:l1norm', ...
            ['L1-norm regularization (qNorm=1) increases QP decision variables.\n' ...
             'Thesis recommends qNorm=2 for real-time applications (Section 5.3.2).']);
    end

    %% Expand Q from per-step (p x p) to full-horizon (p*T_f x p*T_f)
    [qRows, qCols] = size(params.Q);
    if qRows == p && qCols == p
        params.Q_full = kron(eye(T_f), params.Q);
    elseif qRows == p * T_f && qCols == p * T_f
        params.Q_full = params.Q;
    else
        error('Q must be (p x p) or (p*T_f x p*T_f). Got (%d x %d).', qRows, qCols);
    end

    %% Expand R from per-step (m x m) to full-horizon (m*T_f x m*T_f)
    [rRows, rCols] = size(params.R);
    if rRows == m && rCols == m
        params.R_full = kron(eye(T_f), params.R);
    elseif rRows == m * T_f && rCols == m * T_f
        params.R_full = params.R;
    else
        error('R must be (m x m) or (m*T_f x m*T_f). Got (%d x %d).', rRows, rCols);
    end

    %% Expand steady-state references to full horizon
    %  u_r_full = [u_r; u_r; ...; u_r] (m*T_f x 1)
    %  y_r_full = [y_r; y_r; ...; y_r] (p*T_f x 1)
    params.u_r_full = repmat(params.u_r, T_f, 1);
    params.y_r_full = repmat(params.y_r, T_f, 1);

    %% Expand constraint bounds to full horizon
    %  Per-step bounds are tiled T_f times
    if length(params.u_min) == m
        params.u_min_full = repmat(params.u_min, T_f, 1);
    else
        params.u_min_full = params.u_min;
    end

    if length(params.u_max) == m
        params.u_max_full = repmat(params.u_max, T_f, 1);
    else
        params.u_max_full = params.u_max;
    end

    if length(params.y_min) == p
        params.y_min_full = repmat(params.y_min, T_f, 1);
    else
        params.y_min_full = params.y_min;
    end

    if length(params.y_max) == p
        params.y_max_full = repmat(params.y_max, T_f, 1);
    else
        params.y_max_full = params.y_max;
    end

    %% Store dimensions
    params.m = m;
    params.p = p;
end
