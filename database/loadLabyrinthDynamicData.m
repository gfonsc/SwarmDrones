function [U, Y, t, meta] = loadLabyrinthDynamicData(varargin)
% LOADLABYRINTHDYNAMICDATA Load/regenerate the 6-DOF labyrinth DeePC database.

    p = inputParser;
    addParameter(p, 'Regenerate', false, @islogical);
    addParameter(p, 'NumSamples', 520, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Seed', 17, @(x) isnumeric(x) && isscalar(x));
    parse(p, varargin{:});
    opts = p.Results;

    dataDir = fullfile(fileparts(mfilename('fullpath')), 'labyrinth_dynamic');
    dataFile = fullfile(dataDir, 'labyrinth_dynamic_deepc_data.mat');

    if opts.Regenerate || ~isfile(dataFile)
        dataFile = generateLabyrinthDynamicDeePCDatabase( ...
            'OutputDir', dataDir, ...
            'NumSamples', opts.NumSamples, ...
            'Seed', opts.Seed, ...
            'WriteCsv', true);
    end

    S = load(dataFile, 'data');
    data = S.data;

    U = data.U;
    Y = data.Y;
    t = data.t;
    meta = data;

    fprintf('Loaded dynamic labyrinth DeePC database: %d samples, %d inputs, %d outputs.\n', ...
        size(U, 2), size(U, 1), size(Y, 1));
end
