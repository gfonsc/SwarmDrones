function [U, Y, t, meta] = loadBenchmarkMultiData(varargin)
% LOADBENCHMARKMULTIDATA Load multiple IDSIA benchmark runs for DeePC.
%
%   The returned U/Y matrices are concatenated for statistics and scaling.
%   meta.uSegments/meta.ySegments keep the independent runs separated so
%   deepcSetupSegments() can build Hankel columns without crossing CSV
%   boundaries.

    p = inputParser;
    addParameter(p, 'Files', {}, @(x) iscell(x) || isstring(x));
    addParameter(p, 'Patterns', {'random_20251017_run*.csv', ...
        'chirp_20251017_run*.csv', 'square_20251017_run*.csv', ...
        'melon_20251017_run*.csv'}, @iscell);
    addParameter(p, 'StartIdx', 100, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Downsample', 4, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'MaxSamplesPerFile', 110, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'IncludeAttitude', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    dataDir = fullfile(fileparts(mfilename('fullpath')), 'idsia_benchmark');
    files = resolveFiles(dataDir, opts.Files, opts.Patterns);
    assert(~isempty(files), 'No IDSIA CSV files found in %s.', dataDir);

    U = [];
    Y = [];
    t = [];
    uSegments = {};
    ySegments = {};
    tSegments = {};
    names = {};

    for i = 1:numel(files)
        [Ui, Yi, ti] = loadOneIdsiaRun(fullfile(dataDir, files{i}), opts);
        if isempty(Ui)
            continue;
        end

        uSegments{end + 1} = Ui; %#ok<AGROW>
        ySegments{end + 1} = Yi; %#ok<AGROW>
        tSegments{end + 1} = ti; %#ok<AGROW>
        names{end + 1} = files{i}; %#ok<AGROW>

        if isempty(t)
            tOffset = 0;
        else
            tOffset = t(end) + median(diff(ti));
        end
        U = [U, Ui]; %#ok<AGROW>
        Y = [Y, Yi]; %#ok<AGROW>
        t = [t, tOffset + (ti - ti(1))]; %#ok<AGROW>
    end

    meta = struct();
    meta.source = 'IDSIA Nano-Drone System Identification Benchmark';
    meta.dataDir = dataDir;
    meta.files = names;
    meta.uSegments = uSegments;
    meta.ySegments = ySegments;
    meta.tSegments = tSegments;
    meta.dt = median(cellfun(@(x) median(diff(x)), tSegments));
    meta.downsample = opts.Downsample;
    meta.maxSamplesPerFile = opts.MaxSamplesPerFile;
    meta.includeAttitude = opts.IncludeAttitude;
    meta.positionEnvelope = [min(Y(1:3, :), [], 2), max(Y(1:3, :), [], 2)];
    meta.inputEnvelope = [min(U, [], 2), max(U, [], 2)];

    fprintf('Loaded IDSIA multi-run database:\n');
    fprintf('  Runs: %d | Samples: %d | dt: %.3f s | attitude: %d\n', ...
        numel(uSegments), size(U, 2), meta.dt, opts.IncludeAttitude);
    fprintf('  Position envelope x:[%.3f %.3f], y:[%.3f %.3f], z:[%.3f %.3f]\n', ...
        meta.positionEnvelope(1, 1), meta.positionEnvelope(1, 2), ...
        meta.positionEnvelope(2, 1), meta.positionEnvelope(2, 2), ...
        meta.positionEnvelope(3, 1), meta.positionEnvelope(3, 2));
end

function files = resolveFiles(dataDir, explicitFiles, patterns)
    if isstring(explicitFiles)
        explicitFiles = cellstr(explicitFiles);
    end

    if ~isempty(explicitFiles)
        files = explicitFiles(:)';
        return;
    end

    files = {};
    for i = 1:numel(patterns)
        d = dir(fullfile(dataDir, patterns{i}));
        names = sort({d.name});
        files = [files, names]; %#ok<AGROW>
    end
    files = unique(files, 'stable');
end

function [U, Y, t] = loadOneIdsiaRun(filePath, opts)
    raw = readtable(filePath);
    Fs = 100;
    tRaw = raw.t';

    pos = [raw.x, raw.y, raw.z]';
    vel = [raw.vx, raw.vy, raw.vz]';
    rates = [raw.wx, raw.wy, raw.wz]';
    euler = quatToEulerLocal([raw.qx, raw.qy, raw.qz, raw.qw]');
    motors = [raw.m1_rads, raw.m2_rads, raw.m3_rads, raw.m4_rads]';

    [bPos, aPos] = butter(4, 10 / (Fs / 2));
    [bVel, aVel] = butter(4, 18 / (Fs / 2));
    [bMot, aMot] = butter(4, 20 / (Fs / 2));

    posF = filtfilt(bPos, aPos, pos')';
    velF = filtfilt(bVel, aVel, vel')';
    eulerF = filtfilt(bVel, aVel, euler')';
    ratesF = filtfilt(bVel, aVel, rates')';
    motorsF = filtfilt(bMot, aMot, motors')';

    if opts.IncludeAttitude
        Yfull = [posF; velF; eulerF; ratesF];
    else
        Yfull = [posF; velF; ratesF];
    end
    Ufull = motorsToThrustTorques(motorsF);

    idx = opts.StartIdx:round(opts.Downsample):numel(tRaw);
    if isfinite(opts.MaxSamplesPerFile)
        idx = idx(1:min(numel(idx), round(opts.MaxSamplesPerFile)));
    end

    if numel(idx) < 2
        U = [];
        Y = [];
        t = [];
        return;
    end

    U = Ufull(:, idx);
    Y = Yfull(:, idx);
    t = tRaw(idx);
end

function U = motorsToThrustTorques(motors)
    kF = 3.72e-8;
    kM = 7.73e-11;
    L = 0.0325;

    w1 = motors(1, :) .^ 2;
    w2 = motors(2, :) .^ 2;
    w3 = motors(3, :) .^ 2;
    w4 = motors(4, :) .^ 2;

    T = kF * (w1 + w2 + w3 + w4);
    tauX = kF * L * (-w1 - w2 + w3 + w4);
    tauY = kF * L * (-w1 + w2 + w3 - w4);
    tauZ = kM * (-w1 + w2 - w3 + w4);

    U = [T; tauX; tauY; tauZ];
end

function euler = quatToEulerLocal(q)
    qx = q(1, :);
    qy = q(2, :);
    qz = q(3, :);
    qw = q(4, :);

    roll = atan2(2 .* (qw .* qx + qy .* qz), ...
        1 - 2 .* (qx .^ 2 + qy .^ 2));
    pitchArg = 2 .* (qw .* qy - qz .* qx);
    pitchArg = max(-1, min(1, pitchArg));
    pitch = asin(pitchArg);
    yaw = atan2(2 .* (qw .* qz + qx .* qy), ...
        1 - 2 .* (qy .^ 2 + qz .^ 2));

    euler = [roll; pitch; yaw];
end
