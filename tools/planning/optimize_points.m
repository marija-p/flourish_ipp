function obj = optimize_points(waypoints, starting_point, grid_map, ...
     map_parameters, planning_parameters)
% Fitness function for optimizing all points on a horizon for an informative 
% objective

% Create the viewpoints to visit.
waypoints = reshape(waypoints, 3, [])';
waypoints = [starting_point; waypoints];

P_i = trace(grid_map.P);
cost = 0;

% NB: - No measurement taken at first point.
for i = 2:size(waypoints,1)
    point = waypoints(i,:);
    grid_map = predict_map_update(point, grid_map, ...
        map_parameters, planning_parameters);
    cost = cost + max(pdist(waypoints(i-1:i,:)), 5);
end

P_f = trace(grid_map.P);
gain = P_i - P_f;

obj = -gain/cost;

end

