% Script to plot sensor model variance curve
do_print = 1;
paper_pos = [0, 0, 3.2, 2.7];
file_path = '~\PhD\Submissions\asldoc-2017-iros-popovic\images\';

hold on
patch([0 0 10 10], [0 1 1 0], [0.4980, 0.4980, 0.4980], 'FaceAlpha', 0.005, ...
    'EdgeColor', 'none');
patch([10 10 30 30], [0 1 1 0], [0.7500, 0.0980, 0.0980], 'FaceAlpha', 0.04, ...
    'EdgeColor', 'none');
altitude = 0:0.01:26;
var = 0.05 .* (1 - exp(-0.2 .* altitude));
plot(altitude, var, 'Color', [0.0345, 0.0345, 0.0345], 'LineWidth', 1.6)
altitude = [10, 10];
res = [0, 1];
plot(altitude, res, 'Color', [0.7500, 0.0980, 0.0980], 'LineStyle', '--');
hold off
xlabel('Altitude, \ith\rm (m)')
ylabel('Variance, \sigma_{g,i}^2')
axis([0 25 0 0.075])
set(gca, ...
    'Box'         , 'off'     , ...
    'TickDir'     , 'out'     , ...
    'TickLength'  , [.02 .02] , ...
    'XMinorTick'  , 'on'      , ...
    'YMinorTick'  , 'on'      , ...
    'YGrid'       , 'on'      , ...
    'XColor'      , [.3 .3 .3], ...
    'YColor'      , [.3 .3 .3], ...
    'YTick'       , 0:0.025:0.075, ...
    'FontSize'    , 10.5, ...
    'FontName'    , 'Times');

grid minor

set(gcf, 'Position', [2823, 502, 339, 281]);
if (do_print)
    fig = gcf;
    fig.PaperUnits = 'centimeters';
    fig.PaperPosition = paper_pos;
    fig.PaperPositionMode = 'manual';
   % print(fig, '-depscv ', [file_path, 'sensor_model.eps']);
   set(fig,'color','w');
   export_fig sensor_model.eps -painters
end

% close all;