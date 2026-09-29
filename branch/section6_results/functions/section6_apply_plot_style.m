function section6_apply_plot_style()
% SECTION6_APPLY_PLOT_STYLE Apply a clean, academic white-background style
    set(groot, 'defaultFigureColor', 'w');
    set(groot, 'defaultAxesColor', 'w');
    set(groot, 'defaultAxesXColor', 'k');
    set(groot, 'defaultAxesYColor', 'k');
    set(groot, 'defaultAxesZColor', 'k');
    set(groot, 'defaultTextColor', 'k');
    set(groot, 'defaultAxesGridColor', [0.75 0.75 0.75]);
    set(groot, 'defaultAxesMinorGridColor', [0.85 0.85 0.85]);
    set(groot, 'defaultLegendTextColor', 'k');
    set(groot, 'defaultLegendColor', 'w');
    set(groot, 'defaultLineLineWidth', 1.4);
    set(groot, 'defaultAxesFontSize', 11);
    set(groot, 'defaultAxesFontName', 'Times New Roman');
    set(groot, 'defaultTextFontName', 'Times New Roman');
end
