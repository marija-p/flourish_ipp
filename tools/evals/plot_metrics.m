function [] = plot_metrics(metrics)
% Plots informative metrics.

do_plot = 1;

text_size = 10.5;

time_vector = 0:0.1:metrics.times(end);

times = metrics.times;
P_traces = metrics.P_traces;

ts = timeseries(P_traces, times);
ts_resampled = resample(ts, time_vector, 'zoh');
P_traces_resampled = ts_resampled.data';

if (do_plot)
    
    figure;
    hold on

    h_xlabel = xlabel('Time');
    h_ylabel = ylabel('Trace of P');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:100:400, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    plot(time_vector, P_traces_resampled)
    
end

end

