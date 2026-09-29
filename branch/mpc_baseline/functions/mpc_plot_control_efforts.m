function fig = mpc_plot_control_efforts(time, controlSeries, outputBase, plotTitle, channelLabels)
%MPC_PLOT_CONTROL_EFFORTS Plot four control channels for one or more methods.

    if nargin < 5 || isempty(channelLabels)
        channelLabels = {'Channel 1', 'Channel 2', 'Channel 3', 'Channel 4'};
    end

    fig = figure('Color', 'w', 'Position', [120, 120, 1040, 760]);
    tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    for ch = 1:4
        ax = nexttile;
        hold(ax, 'on');
        grid(ax, 'on');
        for i = 1:numel(controlSeries)
            u = coerceControl(controlSeries(i).u);
            tSeries = getSeriesTime(controlSeries(i), time, size(u, 2));
            n = min(numel(tSeries), size(u, 2));
            plot(ax, tSeries(1:n), u(ch, 1:n), 'LineWidth', 1.8, ...
                'Color', controlSeries(i).color, 'LineStyle', controlSeries(i).lineStyle, ...
                'DisplayName', controlSeries(i).name);
        end
        xlabel(ax, 'Time (s)');
        ylabel(ax, channelLabels{ch});
        title(ax, channelLabels{ch});
        if ch == 1
            legend(ax, 'Location', 'best');
        end
    end

    sgtitle(fig, plotTitle, 'FontWeight', 'bold');
    savefig(fig, [outputBase, '.fig']);
    exportgraphics(fig, [outputBase, '.png'], 'Resolution', 200);
end

function u = coerceControl(u)
    if size(u, 1) == 4
        return;
    end
    u = u';
end

function tSeries = getSeriesTime(series, fallbackTime, nSamples)
    if isfield(series, 'time') && ~isempty(series.time)
        tSeries = series.time(:)';
    else
        tSeries = fallbackTime(:)';
    end
    if numel(tSeries) < nSamples
        if numel(tSeries) >= 2
            dt = median(diff(tSeries));
        else
            dt = 1;
        end
        lastValue = tSeries(end);
        tSeries = [tSeries, lastValue + dt * (1:(nSamples - numel(tSeries)))];
    end
end
