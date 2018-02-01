function submap_edge_size = get_submap_edge_size_env(height, ...
    planning_parameters)
% Calculates observation window size from height (env coordinates).
% No discretization issues.

edge_size_x = 2*(height*tand(planning_parameters.sensor_fov_angle_x/2));
edge_size_y = 2*(height*tand(planning_parameters.sensor_fov_angle_y/2));

submap_edge_size.x = edge_size_x;
submap_edge_size.y = edge_size_y;

end
