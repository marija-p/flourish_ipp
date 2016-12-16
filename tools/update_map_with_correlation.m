function grid_map = ...
    update_map_with_correlation(pos_env, submap, ...
    submap_coordinates, grid_map, planning_parameters)
% Updates grid map at a UAV position using measurements
% received using height-dependent sensor model.

%% Update occupancy grid map - weeds.
% Find weeds, extract indices, and update probabilities.
[w_win_j, w_win_i] = find(submap == 1);

w_ind = sub2ind(size(submap), w_win_j, w_win_i);

[sub_submapx, sub_submapy] = meshgrid([submap_coordinates.xl:submap_coordinates.xr],...
        [submap_coordinates.yd:submap_coordinates.yu]);
w_ind_global = sub2ind(size(grid_map.m),reshape(sub_submapy,[],1),reshape(sub_submapx,[],1));
  
var = ones(size(submap));

if (~isempty(w_win_i))
    w_x = submap_coordinates.xl + w_win_i - 1;
    w_y = submap_coordinates.yd + w_win_j - 1;
    w_win_ind = sub2ind(size(grid_map.m), w_y, w_x);
    
    
    submap(w_ind) = classify_weeds(pos_env(3), size(w_win_ind,1), planning_parameters);
    varw = (pos_env(3)/64);
    var(w_ind)=varw;
end

%% Update occupancy grid map - nonweeds.
% Find nonweeds, extract indices, and update probabilities.
[nw_win_j, nw_win_i] = find(submap == 0);

nw_ind = sub2ind(size(submap), nw_win_j, nw_win_i);

if (~isempty(nw_win_i))
    nw_x = submap_coordinates.xl + nw_win_i - 1;
    nw_y = submap_coordinates.yd + nw_win_j - 1;
    nw_win_ind = sub2ind(size(grid_map.m), nw_y, nw_x);
    
    submap(nw_ind) = classify_nonweeds(pos_env(3), size(nw_win_ind,1), planning_parameters);
    varnw = 0.001;
    var(nw_ind)=varnw;
end

%% Perform correlated fusion.

[grid_map.m, grid_map.P] = fuse_measurements(grid_map.m, grid_map.P, ...
    submap,var, w_ind_global);


end

