function fig = section6_plot_xyz(t, ref, deepc, mpc)
% SECTION6_PLOT_XYZ Generate X, Y, Z tracking over time
    fig = figure('Name', 'XYZ Tracking', 'Position', [100, 100, 800, 800]);
    
    labels = {'X Position [m]', 'Y Position [m]', 'Z Position [m]'};
    for i = 1:3
        subplot(3, 1, i);
        hold on;
        if ~isempty(ref)
            plot(t(1:size(ref,1)), ref(:,i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
        end
        if ~isempty(deepc)
            plot(t(1:size(deepc,1)), deepc(:,i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
        end
        if ~isempty(mpc)
            plot(t(1:size(mpc,1)), mpc(:,i), 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
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
end
