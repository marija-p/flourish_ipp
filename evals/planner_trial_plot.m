% Plot results for a single trial with different methods.

%clear all;
close all; clc
show_legend = 1;
methods = fieldnames(logger.trial3);

for i = 2:length(methods)
    
    plot_metrics(logger.trial3.(methods{i}))
    
end

if (show_legend)
    h_legend = legend('No opt.', 'fmincon', ...
        'CMA-ES', ...
        'FontName', 'HelveticaNarrow');
end

% close all;