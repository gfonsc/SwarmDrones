function fig = section6_plot_euler_angles(t, ref_euler, deepc_euler, mpc_euler)
% SECTION6_PLOT_EULER_ANGLES Generate Roll, Pitch, Yaw tracking over time
    fig = figure('Name', 'Euler Angles', 'Position', [100, 100, 800, 800]);
    
    labels = {'Roll [rad]', 'Pitch [rad]', 'Yaw [rad]'};
    for i = 1:3
        subplot(3, 1, i);
        hold on;
        if nargin >= 2 && ~isempty(ref_euler)
            plot(t(1:size(ref_euler,1)), ref_euler(:,i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
        end
        if ~isempty(deepc_euler)
            plot(t(1:size(deepc_euler,1)), deepc_euler(:,i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
        end
        if ~isempty(mpc_euler)
            plot(t(1:size(mpc_euler,1)), mpc_euler(:,i), 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
        end
        ylabel(labels{i});
        if i == 3
            xlabel('Time [s]');
        end
        grid on; box on;
        if i == 1
            legend('Location', 'best');
        end
    end
    section6_apply_plot_style();
    
    if isempty(deepc_euler) && isempty(mpc_euler)
        title(subplot(3,1,1), 'Euler angles unavailable in current data structure');
    end
end
