import iosr.statistics.*

plot_P_trace_final = 1;
plot_P_trace_ave = 0;

close all;

if (plot_P_trace_final)
    
    data = P_traces_cmaes_diff ./ P_traces_coverage_diff;
    figure;
    
    hold on
    h_cov = plot([0 700], [100, 100], 'LineWidth', 1, ...
        'Color', [0.8500, 0.3250, 0.0980]);
    % Shading.
    for i = [60, 160, 260, 360, 460, 560]
        area([i i+80], [200 200], 'FaceAlpha', 0.08, 'LineStyle', 'None', ...
            'FaceColor', [0, 0.2470, 0.4410]);
    end
    
    bp = boxPlot(100:100:600, data*100);
    bp.medianColor = [0, 0.4470, 0.7410];
    bp.symbolMarker = '.';
    bp.lineWidth = 1;
    bp.boxColor = [0.9 0.9 0.9];
    bp.boxAlpha = 1;
    %bp.showMean = true;
    bp.meanColor = [0.4940, 0.1840, 0.5560];
    bp.meanSize = 10;
    %bp.percentile = [15 85];
    axis([0 700 98.5 101])
    xlabel('Path budget (s)')
    ylabel('Relative obj. value - CMA-ES / Cov. (%)')

    pbaspect(gca, [1 1 1])
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'off'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 98.5:0.5:101, ...
        'FontSize'    , 14, ...
        'FontName'    , 'Times', ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
end

if (plot_P_trace_ave)
    
    data = P_traces_ave_cmaes_diff ./ P_traces_ave_coverage_diff;
    figure;
    
    hold on
    h_cov = plot([0 700], [100, 100], 'LineWidth', 1, ...
        'Color', [0.8500, 0.3250, 0.0980]);
    % Shading.
    for i = [60, 160, 260, 360, 460, 560]
        area([i i+80], [200 200], 'FaceAlpha', 0.08, 'LineStyle', 'None', ...
            'FaceColor', [0, 0.2470, 0.4410]);
    end
    
    bp = boxPlot(100:100:600, data*100);
    bp.medianColor = [0, 0.4470, 0.7410];
    bp.symbolMarker = '.';
    bp.lineWidth = 1.2;
    bp.boxColor = [0.9 0.9 0.9];
    bp.boxAlpha = 1;
    %bp.showMean = true;
    bp.meanColor = [0.4940, 0.1840, 0.5560];
    bp.meanSize = 10;
    %bp.percentile = [15 85];
  
    axis([0 700 170 188])
    xlabel('Path budget (s)')
    ylabel({'Relative avg. obj. value - CMA-ES / Cov. (%)'})
    
    pbaspect(gca, [1 1 1])
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'off'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'XTick'       , 100:100:600, ...
        'YTick'       , 170:5:185, ...
        'FontSize'    , 14, ...
        'FontName'    , 'Times', ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    h_legend = legend([bp.handles.box(1,1), h_cov], {'CMA-ES', 'Cov.'}, ...
        'Location', 'southeast');
    
end