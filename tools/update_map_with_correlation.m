function grid_map = ...
    update_map_with_correlation(pos, submap, grid_map, submap_coordinates, ...
    planning_parameters)
% Updates grid map at a UAV position using measurements
% received with height-dependent sensor model.

% Compute variance of measurements.
var = ones(size(submap))*sensor_model(pos(3), planning_parameters);

% Find locations where measurements were taken.
res_factor = get_resolution_from_height(pos(3));

% Altitude resolution diffusion effect:
% Find the starting indices of the "diffused" cells.
if (res_factor ~= 1)
    [submap_ind_x, submap_ind_y] = ...
        meshgrid(submap_coordinates.xl:res_factor:submap_coordinates.xr-1,...
        submap_coordinates.yd:res_factor:submap_coordinates.yu-1);
else
    [submap_ind_x, submap_ind_y] = ...
        meshgrid(submap_coordinates.xl:submap_coordinates.xr,...
        submap_coordinates.yd:submap_coordinates.yu);
end

submap_ind = sub2ind(size(grid_map.m),reshape(submap_ind_y,[],1), ...
    reshape(submap_ind_x,[],1));

%% Perform correlated fusion.
[grid_map.m, grid_map.P] = fuse_measurements(grid_map.m, grid_map.P, ...
    submap, var, submap_ind, pos(3));

%% Perform GP inference.
%[grid_map.m, grid_map.P] = inferece_with_gp(grid_map.m, grid_map.P, ...
%    submap, var, submap_ind);
end

