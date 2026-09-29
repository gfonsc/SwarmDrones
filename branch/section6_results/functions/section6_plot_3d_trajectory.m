function fig = section6_plot_3d_trajectory(ref, deepc, mpc, obstacles, walls)
% SECTION6_PLOT_3D_TRAJECTORY Generate 3D Trajectory plot
    fig = figure('Name', '3D Trajectory', 'Position', [100, 100, 800, 600]);
    hold on;
    
    % Plot reference
    if ~isempty(ref)
        plot3(ref(:,1), ref(:,2), ref(:,3), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
        plot3(ref(1,1), ref(1,2), ref(1,3), 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
        plot3(ref(end,1), ref(end,2), ref(end,3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Finish');
    end
    
    % Plot obstacles
    if nargin >= 4 && ~isempty(obstacles)
        [Xc, Yc, Zc] = cylinder(1, 20);
        for i = 1:size(obstacles, 1)
            obs = obstacles(i,:);
            c = obs(1:2);
            zmin = obs(3);
            zmax = obs(4);
            r = obs(5);
            
            X = c(1) + r * Xc;
            Y = c(2) + r * Yc;
            Z = zmin + (zmax - zmin) * Zc;
            
            surf(X, Y, Z, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        end
        % Dummy for legend
        surf([NaN NaN; NaN NaN], [NaN NaN; NaN NaN], [NaN NaN; NaN NaN], 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Obstacle');
    end
    
    % Plot DeePC
    if ~isempty(deepc)
        plot3(deepc(:,1), deepc(:,2), deepc(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
    end
    
    % Plot MPC
    if ~isempty(mpc)
        plot3(mpc(:,1), mpc(:,2), mpc(:,3), 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
    end
    
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    view(3);
    grid on; box on;
    legend('Location', 'best');
    section6_apply_plot_style();
end
