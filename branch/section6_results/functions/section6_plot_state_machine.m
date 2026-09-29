function fig = section6_plot_state_machine(t, deepc_state, mpc_state)
% SECTION6_PLOT_STATE_MACHINE Generate State Machine activation
    fig = figure('Name', 'State Machine', 'Position', [100, 100, 800, 300]);
    hold on;
    
    if ~isempty(deepc_state)
        stairs(t(1:length(deepc_state)), deepc_state, 'b-', 'LineWidth', 2, 'DisplayName', 'DeePC');
    end
    if ~isempty(mpc_state)
        stairs(t(1:length(mpc_state)), mpc_state, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MPC');
    end
    
    yticks([0 4]);
    yticklabels({'Tracking', 'Avoidance'});
    ylim([-0.5 4.5]);
    xlabel('Time [s]');
    ylabel('State');
    grid on; box on;
    legend('Location', 'best');
    section6_apply_plot_style();
end
