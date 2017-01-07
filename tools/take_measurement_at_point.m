function grid_map = take_measurement_at_point(pos, grid_map, ground_truth_map, ...
    map_parameters, planning_parameters)

% Look at currently observed values based on FoV (camera footprint).
submap_edge_size = ...
    get_submap_edge_size(pos(3), map_parameters, planning_parameters);
submap_coordinates = ...
    get_submap_coordinates(pos, submap_edge_size, map_parameters);
submap = ground_truth_map(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr);

% Downsample submap (measurements) based on altitude.
%submap = get_downsampled_submap(pos(3), submap);

% Update grid map, accounting for correlation.
grid_map = ...
    update_map_with_correlation(pos, submap, grid_map, ...
    submap_coordinates, planning_parameters);

end
