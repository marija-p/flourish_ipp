% Plot results for a single trial with different methods.

%clear all;
close all; clc
show_legend = 1;
methods = fieldnames(logger.trial1);

for i = 1:length(methods)-1
    
    plot_metrics(logger.trial1.(methods{i}))
    
end

if (show_legend)
    h_legend = legend('No opt.', 'CMA-ES', ...
        'fmincon', ...
        'FontName', 'HelveticaNarrow');
end

% close all;