trials = fieldnames(logger);
trials = regexp(trials,'\d*','Match');
trials = [trials{:}];
trials_names = [];
for i = 1:length(trials)
    trials_names = ...
        [trials_names; str2num(cell2mat(trials(i)))];
end

methods = {'cmaes', 'cmaes_nonadaptive'};

%for i = 1:15
for i = 1:length(trials)
   
    % Make sure to use the same random seed!
    t = trials_names(i);
    
    % Generate (continuous) ground truth map.
    rng(t, 'twister');
    cluster_radius = randi([min_cluster_radius, max_cluster_radius]);
    ground_truth_map = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius, 0, 1);
    % Get indices of interesting and non-interesting regions.
    interesting_ind = find(ground_truth_map >= 0.4);
    uninteresting_ind = find(ground_truth_map < 0.4);
    
    for j = 1:length(methods)
        
        load grid_map_init.mat
        
        measurement_points = ...
            logger.((['trial', num2str(t)])).(methods{j}).points_meas;
        logger.(['trial', num2str(t)]).(methods{j}).P_traces_interesting = [];
        logger.(['trial', num2str(t)]).(methods{j}).P_traces_uninteresting = [];
        
        for k = 1:size(measurement_points, 1)
            
            pos = measurement_points(k,:);
            grid_map = take_measurement_at_point(pos, grid_map, ...
                ground_truth_map, map_params, planning_params);
            
            Y_sigma_squared = diag(grid_map.P)';
            Y_sigma_squared = reshape(Y_sigma_squared,40,40);
            P_traces_interesting = ...
                sum(sum(Y_sigma_squared(interesting_ind)));
            P_traces_uninteresting = ...
                sum(sum(Y_sigma_squared(uninteresting_ind)));
            
            logger.((['trial', num2str(t)])).(methods{j}).P_traces_interesting = ...
                [logger.(['trial', num2str(t)]).(methods{j}).P_traces_interesting; ...
                P_traces_interesting];
            logger.((['trial', num2str(t)])).(methods{j}).P_traces_uninteresting = ...
                [logger.(['trial', num2str(t)]).(methods{j}).P_traces_uninteresting; ...
                P_traces_uninteresting];
            
        end
        
    end
    
end