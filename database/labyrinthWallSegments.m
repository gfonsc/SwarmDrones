function wallSegments = labyrinthWallSegments(s)
% LABYRINTHWALLSEGMENTS Return physical wall segments for the labyrinth.
%
% New labyrinth scenarios can define explicit wall geometry. Older scenarios
% still fall back to the corridor union boundary generated from the centerline.

    if isfield(s, 'wallSegments') && ~isempty(s.wallSegments)
        wallSegments = s.wallSegments;
        return;
    end

    corridor = polyshape();
    halfWidth = s.corridorHalfWidth;

    for i = 1:size(s.centerline, 1) - 1
        a = s.centerline(i, :);
        b = s.centerline(i + 1, :);
        d = b - a;
        d = d / max(norm(d), 1e-9);
        n = [-d(2), d(1)];

        rect = [
            a + halfWidth * n;
            b + halfWidth * n;
            b - halfWidth * n;
            a - halfWidth * n
        ];

        corridor = union(corridor, polyshape(rect(:, 1), rect(:, 2), ...
            'Simplify', true));
    end

    [x, y] = boundary(corridor);
    wallSegments = zeros(0, 4);

    for i = 1:numel(x) - 1
        if any(isnan([x(i), y(i), x(i + 1), y(i + 1)]))
            continue;
        end
        a = [x(i), y(i)];
        b = [x(i + 1), y(i + 1)];
        if norm(b - a) > 1e-9
            wallSegments(end + 1, :) = [a, b]; %#ok<AGROW>
        end
    end
end
