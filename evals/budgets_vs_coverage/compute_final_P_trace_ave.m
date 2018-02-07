%trials = fieldnames(logger);
trials = 1:10;
method = 'cmaes';

P_trace_sum = 0;
P_traces = [];

disp(['Number of trials: ', num2str(size(trials,1))]);

for i = 1:length(trials)
    
    %P_traces = [P_traces; ...
    %    logger.(['trial', num2str(i)]).('cmaes').P_traces(end)];
    %P_trace_sum = P_trace_sum + ...
    %    logger.(['trial', num2str(i)]).('cmaes').P_traces(end);    
    P_traces = [P_traces; logger.(trials{i}).(method).P_traces(end)];
    P_trace_sum = P_trace_sum + logger.(trials{i}).(method).P_traces(end);
    
end

P_trace_ave = P_trace_sum / length(trials);

disp('Average Tr(P): ')
disp(P_trace_ave)