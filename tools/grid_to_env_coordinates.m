function [point_env] = grid_to_env_coordinates(point_grid, map_parameters)
% Convert coordinates from grid map to environment representation.

point_env = point_grid;
point_env(:,1) = point_env(:,1) * map_parameters.resolution + map_parameters.position_x;
point_env(:,2) = point_env(:,2) * map_parameters.resolution + map_parameters.position_y;

end
