function fig = section6_plot_2d_trajectory(ref, deepc, mpc, obstacles, walls)
% SECTION6_PLOT_2D_TRAJECTORY Generate 2D Trajectory plot
    fig = figure('Name', '2D Trajectory', 'Position', [100, 100, 800, 600]);
    hold on;
    
    % Plot reference
    if ~isempty(ref)
        plot(ref(:,1), ref(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
        plot(ref(1,1), ref(1,2), 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
        plot(ref(end,1), ref(end,2), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Finish');
    end
    
    % Plot obstacles
    if nargin >= 4 && ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            obs = obstacles(i,:);
            c = obs(1:2);
            r = obs(5);
            r_safe = r + 0.25; % Assumed safe margin
            
            % Plot physical obstacle
            th = linspace(0, 2*pi, 50);
            fill(c(1) + r*cos(th), c(2) + r*sin(th), [0.7 0.7 0.7], 'EdgeColor', 'none', 'HandleVisibility', 'off');
            
            % Plot safety boundary
            plot(c(1) + r_safe*cos(th), c(2) + r_safe*sin(th), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        end
        % Dummy for legend
        fill(NaN, NaN, [0.7 0.7 0.7], 'EdgeColor', 'none', 'DisplayName', 'Obstacle');
        plot(NaN, NaN, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'Safety Bound');
    end
    
    % Plot walls
    if nargin >= 5 && ~isempty(walls)
        for i = 1:size(walls, 1)
            w = walls(i,:);
            plot([w(1) w(3)], [w(2) w(4)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
        end
        plot(NaN, NaN, 'k-', 'LineWidth', 2, 'DisplayName', 'Wall');
    end
    
    % Plot DeePC
    if ~isempty(deepc)
        plot(deepc(:,1), deepc(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
    end
    
    % Plot MPC
    if ~isempty(mpc)
        plot(mpc(:,1), mpc(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
    end
    
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    axis equal;
    grid on; box on;
    legend('Location', 'best');
    section6_apply_plot_style();
end
