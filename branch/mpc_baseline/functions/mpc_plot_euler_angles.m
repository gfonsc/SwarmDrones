function fig = mpc_plot_euler_angles(time, series, outputBase, plotTitle)
%MPC_PLOT_EULER_ANGLES Plot yaw, pitch, and roll when available.

    fig = figure('Color', 'w', 'Position', [120, 120, 980, 760]);
    labels = {'roll (rad)', 'pitch (rad)', 'yaw (rad)'};
    titles = {'Roll', 'Pitch', 'Yaw'};

    for idx = 1:3
        ax = subplot(3, 1, idx);
        hold(ax, 'on');
        grid(ax, 'on');
        plotted = false;
        for i = 1:numel(series)
            euler = coerceEuler(series(i).euler);
            tSeries = getSeriesTime(series(i), time, size(euler, 2));
            n = min(numel(tSeries), size(euler, 2));
            thisSignal = euler(idx, 1:n);
            if all(isnan(thisSignal))
                continue;
            end
            plot(ax, tSeries(1:n), thisSignal, 'LineWidth', 1.8, ...
                'Color', series(i).color, 'LineStyle', series(i).lineStyle, ...
                'DisplayName', series(i).name);
            plotted = true;
        end
        if ~plotted
            text(ax, 0.5, 0.5, 'Unavailable', 'Units', 'normalized', ...
                'HorizontalAlignment', 'center', 'FontAngle', 'italic');
        end
        ylabel(ax, labels{idx});
        title(ax, titles{idx});
        if idx == 1 && plotted
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

function euler = coerceEuler(euler)
    if isempty(euler)
        euler = nan(3, 0);
        return;
    end
    if size(euler, 1) == 3
        return;
    end
    euler = euler';
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
