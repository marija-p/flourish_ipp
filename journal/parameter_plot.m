% Plot results for a single trial with different parameters.

%close all;
clc
show_legend = 1;

figure;
params = fieldnames(logger.trial1.cmaes);

for i = 1:length(params)
    
    plot_metrics(logger.trial1.cmaes.(params{i}));
    
end

% close all;