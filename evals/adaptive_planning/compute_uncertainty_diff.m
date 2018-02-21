trials = fieldnames(logger);
methods = {'none_adaptive', 'none_nonadaptive'};

for i = 16:length(trials)
   
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
        logger.(trials{i}).(methods{j}).uncertainty_diff = [];
        
        for k = 1:size(measurement_points, 1)
            
            pos = measurement_points(k,:);
            grid_map = take_measurement_at_point(pos, grid_map, ...
                ground_truth_map, map_params, planning_params);
            
            Y_sigma_squared = diag(grid_map.P)';
            P_post = reshape(2*Y_sigma_squared,40,40);
            sigma_squared_interesting = mean(mean(P_post(:,1:20)));
            sigma_squared_uninteresting = mean(mean(P_post(:,21:40)));
            sigma_squared_diff = ...
                (sigma_squared_uninteresting - sigma_squared_interesting) / ...
                sigma_squared_uninteresting;
            logger.(trials{i}).(methods{j}).uncertainty_diff = ...
                [logger.(trials{i}).(methods{j}).uncertainty_diff; ...
                sigma_squared_diff];
            
        end
        
    end
    
end