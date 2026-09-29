function [U, Y, t, meta] = loadLabyrinthData(varargin)
% LOADLABYRINTHDATA Load or regenerate the DeePC labyrinth offline database.

    p = inputParser;
    addParameter(p, 'Regenerate', false, @islogical);
    addParameter(p, 'NumSamples', 360, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Seed', 7, @(x) isnumeric(x) && isscalar(x));
    parse(p, varargin{:});
    opts = p.Results;

    dataDir = fullfile(fileparts(mfilename('fullpath')), 'labyrinth');
    dataFile = fullfile(dataDir, 'labyrinth_deepc_data.mat');

    if opts.Regenerate || ~isfile(dataFile)
        dataFile = generateLabyrinthDeePCDatabase( ...
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

    fprintf('Loaded labyrinth DeePC database: %d samples, %d inputs, %d outputs.\n', ...
        size(U, 2), size(U, 1), size(Y, 1));
end
