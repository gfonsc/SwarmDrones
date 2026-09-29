function fig = section6_plot_motor_efforts(t, deepc_u, mpc_u)
% SECTION6_PLOT_MOTOR_EFFORTS Generate Motor 1-4 efforts
    fig = figure('Name', 'Motor Efforts', 'Position', [100, 100, 800, 800]);
    
    for i = 1:4
        subplot(4, 1, i);
        hold on;
        if ~isempty(deepc_u)
            plot(t(1:size(deepc_u,1)), deepc_u(:,i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
        end
        if ~isempty(mpc_u)
            plot(t(1:size(mpc_u,1)), mpc_u(:,i), 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
        end
        ylabel(sprintf('Motor %d', i));
        if i == 4
            xlabel('Time [s]');
        end
        grid on; box on;
        if i == 1
            legend('Location', 'best');
        end
    end
    section6_apply_plot_style();
end
