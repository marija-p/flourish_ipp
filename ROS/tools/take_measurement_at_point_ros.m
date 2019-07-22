function grid_map = take_measurement_at_point_ros(pos, pcl, grid_map, ...
    map_parameters, planning_parameters)

% Project the pointcloud in world frame to the grid
%(including not observed values - NaN).
pcl_map = pcl_to_grid(pcl, map_parameters);

% Find the the boundaries of what is observed.
[obs_idx_y, obs_idx_x] = find(~isnan(pcl_map));
submap_coordinates.xl = min(obs_idx_x);
submap_coordinates.xr = max(obs_idx_x);
submap_coordinates.yd = min(obs_idx_y);
submap_coordinates.yu = max(obs_idx_y);
submap = pcl_map(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr);

% Downsample submap (measurements) based on altitude.
submap = get_downsampled_submap_ros(pos(3), submap);
% Simulate sensor noise effects.
%submap = add_sensor_noise(pos(3), submap, planning_parameters);

% Update grid map, accounting for correlation.
grid_map = ...
    update_map_with_correlation_ros(pos, submap, grid_map, ...
    submap_coordinates, planning_parameters);

end
