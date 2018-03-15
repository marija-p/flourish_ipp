trials = fieldnames(logger);
methods = {'cmaes', 'cmaes_nonadaptive'};

for i = 1:length(trials)
   
   % Generate (continuous) ground truth map.
    rng(i, 'twister');
    cluster_radius = randi([min_cluster_radius, max_cluster_radius]);
    interesting = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius, 0.6, 1);
    uninteresting = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius, 0, 0.4);
    ground_truth_map = interesting;
    ground_truth_map(:, (map_params.dim_y)/2+1:map_params.dim_y) = ...
        uninteresting(:, (map_params.dim_y)/2+1:map_params.dim_y);
    
    for j = 1:length(methods)
        
        load grid_map_init.mat
        
        measurement_points = logger.(trials{i}).(methods{j}).points_meas;
        logger.(trials{i}).(methods{j}).P_traces_interesting = [];
        logger.(trials{i}).(methods{j}).P_traces_uninteresting = [];
        
        for k = 1:size(measurement_points, 1)
            
            pos = measurement_points(k,:);
            grid_map = take_measurement_at_point(pos, grid_map, ...
                ground_truth_map, map_params, planning_params);
            
            Y_sigma_squared = diag(grid_map.P)';
            Y_sigma_squared = reshape(Y_sigma_squared,40,40);
            P_traces_interesting = sum(sum(Y_sigma_squared(:,1:20)));
            P_traces_uninteresting = sum(sum(Y_sigma_squared(:,21:40)));
            
            logger.(trials{i}).(methods{j}).P_traces_interesting = ...
                [logger.(trials{i}).(methods{j}).P_traces_interesting; ...
                P_traces_interesting];
            logger.(trials{i}).(methods{j}).P_traces_uninteresting = ...
                [logger.(trials{i}).(methods{j}).P_traces_uninteresting; ...
                P_traces_uninteresting];
            
        end
        
    end
    
end