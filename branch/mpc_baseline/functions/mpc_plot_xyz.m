function fig = mpc_plot_xyz(time, ref, series, outputBase, plotTitle)
%MPC_PLOT_XYZ Plot x, y, z tracking for one or more methods.

    ref = coerceTrajectory(ref);
    fig = figure('Color', 'w', 'Position', [120, 120, 980, 760]);
    labels = {'x (m)', 'y (m)', 'z (m)'};
    titles = {'x tracking', 'y tracking', 'z tracking'};

    for idx = 1:3
        ax = subplot(3, 1, idx);
        hold(ax, 'on');
        grid(ax, 'on');
        nRef = min(numel(time), size(ref, 1));
        plot(ax, time(1:nRef), ref(1:nRef, idx), 'k--', 'LineWidth', 1.8, 'DisplayName', 'Reference');
        for i = 1:numel(series)
            traj = coerceTrajectory(series(i).traj);
            tSeries = getSeriesTime(series(i), time, size(traj, 1));
            n = min(numel(tSeries), size(traj, 1));
            plot(ax, tSeries(1:n), traj(1:n, idx), 'LineWidth', 1.8, ...
                'Color', series(i).color, 'LineStyle', series(i).lineStyle, ...
                'DisplayName', series(i).name);
        end
        ylabel(ax, labels{idx});
        title(ax, titles{idx});
        if idx == 1
            legend(ax, 'Location', 'best');
        end
        if idx == 3
            xlabel(ax, 'Time (s)');
        end
    end

    sgtitle(fig, plotTitle, 'FontWeight', 'bold');
    savefig(fig, [outputBase, '.fig']);
    exportgraphics(fig, [outputBase, '.png'], 'Resolution', 200);
end

function traj = coerceTrajectory(traj)
    if size(traj, 2) == 3
        return;
    end
    traj = traj';
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
