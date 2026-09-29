function final_plot_motor_efforts_comparison(comparison, outputFile)
%FINAL_PLOT_MOTOR_EFFORTS_COMPARISON Create final normalized effort figure.

    fig = figure('Color', 'w', 'InvertHardcopy', 'off', 'Visible', 'off', ...
        'Position', [80, 50, 1320, 920]);
    tiledlayout(fig, 4, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    for i = 1:4
        plotMotor(nexttile, comparison.scenarioA.deepc, comparison.scenarioA.mpc, i, ...
            sprintf('Scenario A: Motor %d equivalent', i));
        plotMotor(nexttile, comparison.scenarioB.deepc, comparison.scenarioB.mpc, i, ...
            sprintf('Scenario B: Motor %d equivalent', i));
    end

    annotation(fig, 'textbox', [0.12, 0.005, 0.78, 0.035], ...
        'String', 'Equivalent normalized motor effort from branch command channels; not physical motor speed.', ...
        'Color', 'k', 'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 9);
    exportgraphics(fig, outputFile, 'Resolution', 300);
    close(fig);
end

function plotMotor(ax, deepcResult, mpcResult, idx, titleText)
    hold(ax, 'on'); grid(ax, 'on');
    styleAxes(ax);
    t = deepcResult.log.time;
    plot(ax, t, deepcResult.motorEffort.effort(idx, :), 'Color', [0.10, 0.36, 0.80], ...
        'LineWidth', 1.5, 'DisplayName', 'DeePC');
    plot(ax, t, mpcResult.motorEffort.effort(idx, :), 'Color', [0.82, 0.20, 0.18], ...
        'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', 'MPC');
    xlabel(ax, 'time (s)', 'Color', 'k');
    ylabel(ax, 'normalized effort', 'Color', 'k');
    ylim(ax, [-1.05, 1.05]);
    title(ax, titleText, 'FontWeight', 'bold', 'Color', 'k');
    if idx == 1
        leg = legend(ax, 'Location', 'best');
        styleLegend(leg);
    end
end

function styleAxes(ax)
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.72, 0.72, 0.72], 'GridAlpha', 0.35, ...
        'FontSize', 9.5, 'LineWidth', 0.8, 'Box', 'on');
    try
        disableDefaultInteractivity(ax);
        ax.Toolbar.Visible = 'off';
    catch
    end
end

function styleLegend(leg)
    set(leg, 'Color', 'w', 'TextColor', 'k', 'EdgeColor', [0.35, 0.35, 0.35]);
end
