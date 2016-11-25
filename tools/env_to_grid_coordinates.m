function [point_grid] = env_to_grid_coordinates(point_env, map_parameters)
% Convert coordinates from environment to occupancy grid map
% representation.

point_grid = point_env;
point_grid(:,1) = round((point_grid(:,1) + map_parameters.env_dim_y/2) * ...
    1/map_parameters.resolution);
point_grid(:,2) = round((point_grid(:,2) + map_parameters.env_dim_x/2) * ...
    1/map_parameters.resolution);

end

