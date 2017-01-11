function [lattice] = create_lattice(map_parameters, planning_parameters, ...
    min_height_points, height_increment)
% Create multi-dimensional lattice in the UAV configuraton space.

lattice = [];

for height = planning_parameters.min_height:height_increment:planning_parameters.max_height
    
    point_coeffs = polyfit([min_height_points, 1], ...
        [planning_parameters.min_height, planning_parameters.max_height], 1);
    num_of_points = point_coeffs(1)*height + point_coeffs(2);
    
    submap_edge_size = get_submap_edge_size(height,  map_parameters, planning_parameters);
    half_submap_edge_size_x = (submap_edge_size.x-1)/2;
    half_submap_edge_size_y = (submap_edge_size.y-1)/2;
    
    % Compute distance between points on a lattice plane,
    % assuming same discretisation in x- and y-dirs.
    [grid_x, grid_y] = meshgrid(linspace(half_submap_edge_size_x, map_parameters.dim_x-half_submap_edge_size_x, sqrt(num_of_points)), ...
                        linspace(half_submap_edge_size_y, map_parameters.dim_y-half_submap_edge_size_y, sqrt(num_of_points)));
    grid_x = reshape(grid_x, [], 1);
    grid_y = reshape(grid_y, [], 1);
    grid_z = height*ones(size(grid_x,1),1);
    
    % Add grid at current altitude level to lattice.
    lattice = [lattice; grid_x, grid_y, grid_z];
    
end

lattice = grid_to_env_coordinates(lattice, map_parameters);

%plot3(lattice(:,1), lattice(:,2), lattice(:,3), '.k');
 
end
