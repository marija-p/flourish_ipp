function obj = optimize_points_bo(waypoints_var, starting_point, grid_map, ...
     map_parameters, planning_parameters)
% Fitness function for optimizing all points on a horizon for an informative 
% objective

waypoints = [waypoints_var.x1, waypoints_var.y1, waypoints_var.z1; ...
             waypoints_var.x2, waypoints_var.y2, waypoints_var.z2; ...
             waypoints_var.x3, waypoints_var.y3, waypoints_var.z3; ...
             waypoints_var.x4, waypoints_var.y4, waypoints_var.z4];
waypoints = [starting_point; waypoints];

obj = compute_objective(waypoints, grid_map, map_parameters, planning_parameters);

end

