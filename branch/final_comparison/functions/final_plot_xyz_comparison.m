function final_plot_xyz_comparison(comparison, outputFile)
%FINAL_PLOT_XYZ_COMPARISON Create the final x/y/z paper figure.

    fig = figure('Color', 'w', 'InvertHardcopy', 'off', 'Visible', 'off', ...
        'Position', [80, 60, 1320, 840]);
    tiledlayout(fig, 3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
    labels = {'x (m)', 'y (m)', 'z (m)'};
    titles = {'x position', 'y position', 'z position'};

    for i = 1:3
        plotAxis(nexttile, comparison.scenarioA.deepc, comparison.scenarioA.mpc, i, ...
            ['Scenario A: ', titles{i}], labels{i});
        plotAxis(nexttile, comparison.scenarioB.deepc, comparison.scenarioB.mpc, i, ...
            ['Scenario B: ', titles{i}], labels{i});
    end

    exportgraphics(fig, outputFile, 'Resolution', 300);
    close(fig);
end

function plotAxis(ax, deepcResult, mpcResult, idx, titleText, yLabel)
    hold(ax, 'on'); grid(ax, 'on');
    styleAxes(ax);
    t = deepcResult.log.time;
    plot(ax, t, deepcResult.reference.position(idx, :), 'k--', 'LineWidth', 1.6, 'DisplayName', 'Reference');
    plot(ax, t, deepcResult.log.pos(idx, :), 'Color', [0.10, 0.36, 0.80], ...
        'LineWidth', 1.7, 'DisplayName', 'DeePC');
    plot(ax, t, mpcResult.log.pos(idx, :), 'Color', [0.82, 0.20, 0.18], ...
        'LineWidth', 1.7, 'LineStyle', '--', 'DisplayName', 'MPC');
    xlabel(ax, 'time (s)', 'Color', 'k');
    ylabel(ax, yLabel, 'Color', 'k');
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
