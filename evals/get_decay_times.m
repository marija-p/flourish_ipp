close all;

% Script to find mission time at which map uncertainty decays
% to a particular percentage for several trials.

import iosr.statistics.*

if (isfield(logger, 'trial20'))
    logger = rmfield(logger, 'trial20');
end
if (isfield(logger, 'trial31'))
    logger = rmfield(logger, 'trial31');
end
if (isfield(logger, 'trial30'))
    logger = rmfield(logger, 'trial30');
end
if (isfield(logger, 'trial21'))
    logger = rmfield(logger, 'trial21');
end

do_plot = 1;

P_trace_init = 339.3582;
methods = {'none', 'cmaes', 'fmc', 'sa', 'bo', 'rig', 'random', 'coverage'};
trials = fieldnames(logger);
decay_percent = 0.25;

decay_times = zeros(length(trials),length(methods));

for i = 1:length(trials)
    
    for j = 1:length(methods)
    
        ind = ...
            find(logger.(trials{i}).([methods{j}]).P_traces < ...
            P_trace_init*decay_percent);
        decay_times(i,j) = ...
            logger.(trials{i}).([methods{j}]).times(ind(1));
        
    end
    
end

disp(decay_times)

if (do_plot)
    hold on
    % Shading.
    for i = [0.6, 1.6, 2.6, 3.6, 4.6, 5.6, 6.6, 7.6]
        area([i i+0.8], [200 200], 'FaceAlpha', 0.08, 'LineStyle', 'None', ...
            'FaceColor', [0, 0.2470, 0.4410]);
    end
    bp = boxPlot({'Lattice', 'CMA-ES', 'IP', 'SA', 'BO', 'RIG-tree', ...
        'Random', 'Coverage'}, decay_times);
    bp.medianColor = [0, 0.4470, 0.7410];
    bp.symbolMarker = '.';
    bp.lineWidth = 1;
    bp.boxWidth = 0.5;
    bp.boxColor = [0.9 0.9 0.9];
    bp.boxAlpha = 1;
    %bp.showMean = true;
    bp.meanColor = [0.4940, 0.1840, 0.5560];
    bp.meanSize = 10;
    bp.percentile = [15 85];
    axis([0.5 8.5 0 160])
    ylabel('Time (s)')
    pbaspect(gca, [2.5 1 1])
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'off'      , ...
        'YMinorTick'  , 'off'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:25:150, ...
        'FontSize'    , 14, ...
        'FontName'    , 'Times', ...
        'LooseInset'  , max(get(gca,'TightInset'), 0.02));
end