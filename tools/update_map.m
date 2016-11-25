function [grid_map] = ...
    update_map(pos_env, submap, submap_coordinates, grid_map, planning_parameters)
% Updates occupancy grid map at a UAV position using measurements
% received using height-dependent sensor model.

%% Update occupancy grid map - weeds.
% Find weeds, extract indices, and update probabilities.
[w_win_j, w_win_i] = find(submap == 1);

if (~isempty(w_win_i))
    w_x = submap_coordinates.xl + w_win_i - 1;
    w_y = submap_coordinates.yd + w_win_j - 1;
    w_win_ind = sub2ind(size(grid_map), w_y, w_x);
    grid_map(w_win_ind) = grid_map(w_win_ind) + ...
        prob_to_logodds(classify_weeds(pos_env(3), size(w_win_ind,1), planning_parameters));
end

%% Update occupancy grid map - nonweeds.
% Find nonweeds, extract indices, and update probabilities.
[nw_win_j, nw_win_i] = find(submap == 0);

if (~isempty(nw_win_i))
    nw_x = submap_coordinates.xl + nw_win_i - 1;
    nw_y = submap_coordinates.yd + nw_win_j - 1;
    nw_win_ind = sub2ind(size(grid_map), nw_y, nw_x);
    grid_map(nw_win_ind) = grid_map(nw_win_ind) + ...
        prob_to_logodds(classify_nonweeds(pos_env(3), size(nw_win_ind,1), planning_parameters));
end

end

