% Plot results for a single trial with different methods.

%close all;
clc
show_legend = 1;

figure;
methods = fieldnames(logger.trial1);

for i = 2:length(methods)
    
    plot_metrics(logger.trial1.(methods{i}))
    
end

if (show_legend)
    h_legend = legend('No opt.', 'CMA-ES', ...
        'fmincon', 'RIG-tree', 'Coverage');
end

% close all;