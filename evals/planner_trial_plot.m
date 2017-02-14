% Plot results for a single trial with different methods.

%clear all;
close all; clc
show_legend = 1;
methods = fieldnames(logger.trial2);

for i = 1:length(methods)-1
    
    plot_metrics(logger.trial2.(methods{i}))
    
end

if (show_legend)
    h_legend = legend('No opt.', 'fmincon', ...
        'CMA-ES', ...
        'FontName', 'HelveticaNarrow');
end

% close all;