function grid_map = ...
    predict_map_update(pos, grid_map, map_parameters, planning_parameters)
% Predicts grid map update at an unvisited UAV position using
% height-dependent sensor model.
% --
% Inputs:
% pos: current [x,y,z] UAV position [m] - env. coordinates
% grid_map: current grid map (mean + covariance)
% ---
% Outputs:
% grid map
% ---
% M Popovic 2017
%

%% Measurement Prediction
% Compute current FoV (camera footprint).
submap_edge_size = ...
    get_submap_edge_size(pos(3), map_parameters, planning_parameters);
submap_coordinates = ...
    get_submap_coordinates(pos, submap_edge_size, map_parameters);
submap = grid_map.m(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr);

%% Sensor Model
% Compute measurement variances.
var = ones(size(submap))*sensor_model(pos(3), planning_parameters);
R = diag(reshape(var,[],1));
% Create measurement model.
H = construct_H(grid_map.m, submap, submap_coordinates, pos(3));

%% Covariance Update
grid_map.P = KF_update_cholesky(grid_map.P, R, H);

%%TODO:- GP inference function here?

end