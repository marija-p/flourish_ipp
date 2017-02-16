function obj = optimize_points(waypoints, starting_point, grid_map, ...
     map_parameters, planning_parameters)
% Fitness function for optimizing all points on a horizon for an informative 
% objective

waypoints = reshape(waypoints, 3, [])';
waypoints = [starting_point; waypoints];

obj = compute_objective(waypoints, grid_map, map_parameters, planning_parameters);

end