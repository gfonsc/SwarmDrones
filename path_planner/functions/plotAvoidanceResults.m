function plotAvoidanceResults(trajectory, stateHistory, yawHistory, ...
    obstacles, goal, start, plannerConstants, options)
% PLOTAVOIDANCERESULTS  Visualize path planner simulation results.
%
%   plotAvoidanceResults(trajectory, stateHistory, yawHistory, ...
%       obstacles, goal, start, plannerConstants)
%   plotAvoidanceResults(..., options)
%
%   Creates a multi-panel figure showing:
%     (a) Top-down view with trajectory color-coded by state
%     (b) 3D view with obstacle cylinders
%     (c) State timeline (lateral + altitude FSM)
%     (d) Distance to goal over time
%
%   Inputs:
%       trajectory       - Position history (3 x N)
%       stateHistory     - Lateral state IDs (1 x N)
%       yawHistory       - Yaw angle history (1 x N)
%       obstacles        - Obstacle matrix (M x 5)
%       goal             - Goal position (3 x 1)
%       start            - Start position (3 x 1)
%       plannerConstants - constants struct from planner
%       options          - (Optional) Struct with:
%                           .Ts            - Sampling period [s]
%                           .title         - Figure title (string)
%                           .savePath      - File path to save figure
%                           .collisionPts  - Collision positions (3 x K)
%                           .altHistory    - Altitude state IDs (1 x N)

    if nargin < 8
        options = struct();
    end

    Ts = getOpt(options, 'Ts', 0.01);
    figTitle = getOpt(options, 'title', 'Path Planner — Avoidance Results');
    savePath = getOpt(options, 'savePath', '');
    collisionPts = getOpt(options, 'collisionPts', []);
    altHistory = getOpt(options, 'altHistory', []);

    N = size(trajectory, 2);
    t = (0:N-1) * Ts;

    stateNames = plannerConstants.stateNames;
    nStates = length(stateNames);

    %% State color map
    stateColors = [
        0.2, 0.6, 1.0;   % ALIGN         - blue
        0.1, 0.8, 0.2;   % GO_TO_GOAL    - green
        1.0, 0.7, 0.0;   % ORIENT        - orange
        0.8, 0.4, 0.0;   % TRANSLATE     - dark orange
        0.7, 0.0, 0.7;   % LAT_ESCAPE    - purple
        0.0, 0.9, 0.5;   % AT_GOAL       - teal
        1.0, 0.0, 0.0;   % COLLISION     - red
    ];

    %% Obstacle visualization helper
    theta_c = linspace(0, 2*pi, 40);
    nObs = size(obstacles, 1);

    %% Create figure
    fig = figure('Name', figTitle, 'Position', [50, 50, 1400, 700], 'Color', 'w');

    % ===== (a) Top-down view =====
    subplot(2, 2, 1);
    hold on;

    % Draw obstacles
    for i = 1:nObs
        cx = obstacles(i,1) + obstacles(i,5) * cos(theta_c);
        cy = obstacles(i,2) + obstacles(i,5) * sin(theta_c);
        z_max = obstacles(i, 4);
        if z_max < 1.0
            fill(cx, cy, [0.5, 0.9, 0.5], 'EdgeColor', [0.3, 0.6, 0.3], 'FaceAlpha', 0.7);
        elseif obstacles(i,3) > 1.5
            fill(cx, cy, [1.0, 1.0, 0.5], 'EdgeColor', [0.7, 0.7, 0.3], 'FaceAlpha', 0.7);
        else
            fill(cx, cy, [0.6, 0.6, 0.6], 'EdgeColor', [0.4, 0.4, 0.4], 'FaceAlpha', 0.7);
        end
    end

    % Draw trajectory colored by state
    for s = 1:nStates
        mask = stateHistory == s;
        if any(mask)
            scatter(trajectory(1, mask), trajectory(2, mask), 8, ...
                stateColors(s, :), 'filled', 'DisplayName', stateNames{s});
        end
    end

    % Start/Goal
    plot(start(1), start(2), 'go', 'MarkerSize', 14, 'MarkerFaceColor', 'g', ...
        'LineWidth', 2, 'DisplayName', 'Start');
    plot(goal(1), goal(2), 'rp', 'MarkerSize', 16, 'MarkerFaceColor', 'r', ...
        'LineWidth', 2, 'DisplayName', 'Goal');

    % Collisions
    if ~isempty(collisionPts)
        plot(collisionPts(1,:), collisionPts(2,:), 'rx', 'MarkerSize', 20, ...
            'LineWidth', 3, 'DisplayName', 'Collision');
    end

    xlabel('X (m)'); ylabel('Y (m)');
    title('(a) Top View — Color = State');
    legend('Location', 'bestoutside', 'FontSize', 7);
    axis equal; grid on;

    % ===== (b) 3D view =====
    subplot(2, 2, 2);
    hold on;

    for i = 1:nObs
        [X, Y, Z] = cylinder(obstacles(i,5), 20);
        Z = Z * (obstacles(i,4) - obstacles(i,3)) + obstacles(i,3);
        z_max = obstacles(i,4);
        if z_max < 1.0
            col = [0.5, 0.9, 0.5];
        elseif obstacles(i,3) > 1.5
            col = [1.0, 1.0, 0.5];
        else
            col = [0.6, 0.6, 0.6];
        end
        surf(X + obstacles(i,1), Y + obstacles(i,2), Z, ...
            'FaceColor', col, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    end

    plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), ...
        'b-', 'LineWidth', 1.5);
    plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 12, ...
        'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot3(goal(1), goal(2), goal(3), 'rp', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'r', 'LineWidth', 2);

    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('(b) 3D Trajectory');
    view(45, 30); axis equal; grid on;

    % ===== (c) State timeline =====
    subplot(2, 2, 3);
    hold on;

    % Color bars for each state
    for k = 1:N-1
        s_id = stateHistory(k);
        if s_id >= 1 && s_id <= nStates
            fill([t(k), t(k+1), t(k+1), t(k)], [0, 0, 1, 1], ...
                stateColors(s_id, :), 'EdgeColor', 'none');
        end
    end

    % Altitude state (on top half if available)
    if ~isempty(altHistory)
        altColors = [0.8, 0.8, 0.8; 0.2, 0.7, 1.0; 1.0, 0.5, 0.2];
        for k = 1:N-1
            a_id = altHistory(k) + 1;  % 0-indexed to 1-indexed
            if a_id >= 1 && a_id <= 3
                fill([t(k), t(k+1), t(k+1), t(k)], [1.1, 1.1, 1.5, 1.5], ...
                    altColors(a_id, :), 'EdgeColor', 'none');
            end
        end
        ylim([-0.1, 1.6]);
        text(t(end)*0.02, 1.3, 'Altitude FSM', 'FontSize', 8, 'FontWeight', 'bold');
    else
        ylim([-0.1, 1.1]);
    end

    xlabel('Time (s)');
    title('(c) State Timeline');
    set(gca, 'YTick', []);
    grid on;

    % Legend (one entry per state)
    for s = 1:nStates
        plot(NaN, NaN, 's', 'MarkerFaceColor', stateColors(s,:), ...
            'MarkerEdgeColor', 'none', 'MarkerSize', 8, ...
            'DisplayName', stateNames{s});
    end
    legend('Location', 'bestoutside', 'FontSize', 7);

    % ===== (d) Distance to goal =====
    subplot(2, 2, 4);
    dist2goal = vecnorm(trajectory - goal, 2, 1);
    plot(t, dist2goal, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(0.25, 'g--', 'LineWidth', 1, 'DisplayName', 'Goal tolerance');
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('(d) Distance to Goal');
    legend('Location', 'best'); grid on;

    %% Super title
    sgtitle(figTitle, 'FontSize', 14, 'FontWeight', 'bold');

    %% Save
    if ~isempty(savePath)
        exportgraphics(fig, savePath, 'Resolution', 200);
        fprintf('  Figure saved: %s\n', savePath);
    end
end

%% Helper: get optional field
function v = getOpt(s, field, default)
    if isfield(s, field)
        v = s.(field);
    else
        v = default;
    end
end
