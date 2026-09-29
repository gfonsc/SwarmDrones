function fig = section6_plot_safety_margin(t, deepc_h, mpc_h)
% SECTION6_PLOT_SAFETY_MARGIN Generate h(k) safety margin
    fig = figure('Name', 'Safety Margin', 'Position', [100, 100, 800, 400]);
    hold on;
    
    if ~isempty(deepc_h)
        plot(t(1:length(deepc_h)), deepc_h, 'b-', 'LineWidth', 1.5, 'DisplayName', 'DeePC');
    end
    if ~isempty(mpc_h)
        plot(t(1:length(mpc_h)), mpc_h, 'r-', 'LineWidth', 1.5, 'DisplayName', 'MPC');
    end
    
    % Zero line
    plot([t(1) t(end)], [0 0], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Safety Threshold (h=0)');
    
    % Safe region background
    y_lim = ylim;
    fill([t(1) t(end) t(end) t(1)], [0 0 y_lim(2) y_lim(2)], [0.9 1 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');
    fill([t(1) t(end) t(end) t(1)], [y_lim(1) y_lim(1) 0 0], [1 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');
    
    % Bring lines to front
    children = get(gca, 'Children');
    set(gca, 'Children', circshift(children, -2));
    
    xlabel('Time [s]');
    ylabel('Safety Margin h(k) [m]');
    grid on; box on;
    legend('Location', 'best');
    section6_apply_plot_style();
end
