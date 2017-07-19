function grid_map = ...
    update_map_with_correlation(pos, submap, grid_map, submap_coordinates, ...
    planning_parameters)
% Updates grid map at a UAV position using measurements
% received from a height-dependent sensor model.
%
% Inputs:
% pos: current [x,y,z] UAV position [m] - env. coordinates
% submap: received measurement (grid)
% grid_map: current grid map (mean + covariance)
% submap_coordinates: current FoV in grid map
% ---
% Outputs:
% grid_map
% ---
% Marija Popovic 2017
%

%% Sensor Model
% Compute measurement variances.
var = ones(size(submap))*sensor_model(pos(3), planning_parameters);
R = diag(reshape(var,[],1));
% Create measurement model.
H = construct_H(grid_map.m, submap, submap_coordinates, pos(3), 0);

%% Correlated Fusion
% Obtain maximum aposteriori estimate using Bayesian Fusion.
x = reshape(grid_map.m,[],1);
z = reshape(submap,[],1);
P = grid_map.P;
[x,Pf] = KF_update_cholesky(x,P,z-H*x,R,H);
grid_map.m = reshape(x, size(grid_map.m,1), size(grid_map.m,2));
grid_map.P = Pf;

end

