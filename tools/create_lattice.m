function [lattice_env] = create_lattice(map_parameters, planning_parameters)
% Create multi-dimensional lattice in the UAV configuraton space.

lattice = [];

manual_lattice = 0;

if (~manual_lattice)
    
    point_coeffs = polyfit([planning_parameters.min_height, planning_parameters.max_height], ...
        [planning_parameters.lattice_min_height_points, 1], 1);
    
    for height = planning_parameters.min_height: ...
            planning_parameters.lattice_height_increment: ...
            planning_parameters.max_height
        
        num_of_points = round(point_coeffs(1)*height + point_coeffs(2));
        
        submap_edge_size = ...
            get_submap_edge_size(height, map_parameters, planning_parameters);
        half_submap_edge_size_x = (submap_edge_size.x-1)/2;
        half_submap_edge_size_y = (submap_edge_size.y-1)/2;
        % Compute distance between points on a lattice plane,
        % assuming same discretisation in x- and y-dirs.
        [grid_x, grid_y] = meshgrid(linspace(half_submap_edge_size_x, ...
            map_parameters.dim_x-half_submap_edge_size_x, sqrt(num_of_points)), ...
            linspace(half_submap_edge_size_y, ...
            map_parameters.dim_y-half_submap_edge_size_y, sqrt(num_of_points)));
        grid_x = reshape(grid_x, [], 1);
        grid_y = reshape(grid_y, [], 1);
        grid_z = height*ones(size(grid_x,1),1);
        
        % Add grid at current altitude level to lattice.
        lattice = [lattice; grid_x, grid_y, grid_z];
        
    end
    
    %lattice = grid_to_env_coordinates(lattice, map_parameters);
    lattice_env = grid_to_env_coordinates(lattice, map_parameters);
    
    %plot3(lattice_env(:,1), lattice_env(:,2), lattice_env(:,3), '.k');
    
else
    
    lattice_env = [-14.5858, -14.4226, 1;
        -14.5858, -4.80755, 1;
        -14.5858, 4.80755, 1;
        -14.5858, 14.4226, 1;
        -4.86193, -14.4226, 1;
        -4.86193, -4.80755, 1;
        -4.86193, 4.80755, 1;
        -4.86193, 14.4226, 1;
        4.86193, -14.4226, 1;
        4.86193, -4.80755, 1;
        4.86193, 4.80755, 1;
        4.86193, 14.4226, 1;
        14.5858, -14.4226, 1;
        14.5858, -4.80755, 1;
        14.5858, 4.80755, 1;
        14.5858, 14.4226, 1;
        -11.2721, -9.80385, 9;
        -11.2721, 0, 9;
        -11.2721, 9.80385, 9;
        0, -9.80385, 9;
        0, 0, 9;
        0, 9.80385, 9;
        11.2721, -9.80385, 9;
        11.2721, 0, 9;
        11.2721, 9.80385, 9;
        -7.95837, -5.18505, 17;
        -7.95837, 5.18505, 17;
        7.95837, -5.18505, 17;
        7.95837, 5.18505, 17;
        0, 0, 25];
    
end

end
