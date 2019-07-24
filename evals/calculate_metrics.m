 if (isfield(logger, 'trial20'))
     logger = rmfield(logger, 'trial20');
 end
 if (isfield(logger, 'trial30'))
     logger = rmfield(logger, 'trial30');
 end
 if (isfield(logger, 'trial21'))
     logger = rmfield(logger, 'trial21');
 end
% if (isfield(logger, 'trial12'))
%     logger = rmfield(logger, 'trial12');
% end

trials_names = fieldnames(logger);
methods_names = fieldnames(logger.trial1);
methods_names(1) = [];
metrics_names = {'P_traces', 'rmses', 'wrmses', 'mlls', 'wmlls'};

metrics = zeros(length(trials_names),length(methods_names),length(metrics_names));

for i = 1:length(trials_names)
    
    for j = 1:length(methods_names)
        
        for k = 1:length(metrics_names)
            
            try
                metrics(i,j,k,:) = ...
                    mean(logger.(trials_names{i}).(methods_names{j}).(metrics_names{k});
            catch
                disp(['Cannot find: ', trials_names{i}, ' ', ...
                    methods_names{j}, ' ', metrics_names{k}]);
            end
            
        end
        
    end
    
end


disp(['Number of trials: ', num2str(length(trials_names))])
squeeze(mean(metrics,1))