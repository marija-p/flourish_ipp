function grid_map = take_measurement_at_point(pos, grid_map_mean, ground_truth_map, ...
    map_parameters, planning_parameters, grid_map_cov)

% Look at currently observed values based on FoV (camera footprint).
submap_edge_size = ...
    get_submap_edge_size(pos(3), map_parameters, planning_parameters);
submap_coordinates = ...
    get_submap_coordinates(pos, submap_edge_size, map_parameters);
submap = ground_truth_map(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr);

switch nargin
    case 5
    % Update occupancy grid with received measurements and classifier model.
    grid_map = ...
        update_map(pos, submap, submap_coordinates, grid_map_mean, planning_parameters);
    
    case 6
    % Update gridmap with correlation
    grid_map = ...
        update_map_with_correlation(pos, submap, submap_coordinates, grid_map_mean, planning_parameters, grid_map_cov);
    
    otherwise
        display('Input parameters are missing');
end
end

