function section6_export_figure(fig, outputPath)
% SECTION6_EXPORT_FIGURE Export a figure as PNG, FIG, and PDF with a white background.
    
    % Ensure figure has white background
    set(fig, 'Color', 'w');
    axesList = findall(fig, 'type', 'axes');
    for i=1:length(axesList)
        set(axesList(i), 'Color', 'w');
        grid(axesList(i), 'on');
    end

    % Define file prefixes
    [fdir, fname, ~] = fileparts(outputPath);
    if ~exist(fdir, 'dir')
        mkdir(fdir);
    end
    
    basePath = fullfile(fdir, fname);
    
    % Export PNG
    try
        exportgraphics(fig, [basePath '.png'], 'BackgroundColor', 'white', 'Resolution', 300);
    catch
        print(fig, [basePath '.png'], '-dpng', '-r300');
    end
    
    % Export FIG
    savefig(fig, [basePath '.fig']);
    
    % Export PDF
    try
        exportgraphics(fig, [basePath '.pdf'], 'BackgroundColor', 'white', 'ContentType', 'vector');
    catch
        print(fig, [basePath '.pdf'], '-dpdf', '-vector');
    end
end
