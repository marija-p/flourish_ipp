% Example of an update step of an unknown map using a single sensor
% measurement from a fixed altitude.
% Conventional occupancy grid mapping.

%% Parameters %%
% Camera fields of view (FoV)
planning_parameters.classifier_fov_angle_x = 60;
planning_parameters.classifier_fov_angle_y = 60;
% Sensor models: polynomial coefficients
planning_parameters.weed_coeffs = [-0.000256704980842912, -0.00273180076628354, 0.912988505747127];
planning_parameters.nonweed_coeffs = [0.000233716475095785, -0.00134865900383140, 0.130114942528736];

% Map resolution [m/cell]
map_parameters.resolution = 0.5;
% Map dimensions (from ground truth)
map_parameters.env_dim_x = 10;
map_parameters.env_dim_y = 10;

% Create plots.
matlab_parameters.visualize = 1;

%% Initialization %%
% Create a ground truth weed map.
[ground_truth_map] = ...
    create_poisson_map(25, ...
    map_parameters.env_dim_x/map_parameters.resolution, ...
    map_parameters.env_dim_y/map_parameters.resolution);
% Set position of UAV in the environment (m).
pos_env = [0, 0, 2];
% Create occupancy grid map with unknown values.
grid_map = 0.5*ones(size(ground_truth_map));
% Convert to log-odds.
grid_map = prob_to_logodds(grid_map);

if (matlab_parameters.visualize)
    subplot(1,3,1)
    imagesc(ground_truth_map);
    title('Ground truth weed map')
    set(gca,'Ydir','Normal');
    
    subplot(1,3,2)
    imagesc(logodds_to_prob(grid_map));
    title('Weed map before update')
    set(gca,'Ydir','Normal');
end

% Look at currently observed values based on FoV (camera footprint).
submap_edge_size = ...
    get_submap_edge_size(pos_env(3), map_parameters, planning_parameters);
submap_coordinates = ...
    get_submap_coordinates(pos_env, submap_edge_size, map_parameters);
submap = ground_truth_map(submap_coordinates.yd:submap_coordinates.yu, ...
    submap_coordinates.xl:submap_coordinates.xr);

% Update occupancy grid with received measurements and classifier model.
grid_map = ...
    update_map(pos_env, submap, submap_coordinates, grid_map, planning_parameters);

if (matlab_parameters.visualize)
    subplot(1,3,3)
    imagesc(logodds_to_prob(grid_map));
    title('Weed map after update')
    set(gca,'Ydir','Normal');
    
    set (gcf, 'Units', 'normalized', 'Position', [0.0297, 0.2620, 0.9672, 0.4741]);
    c = colorbar;
    caxis([0, 1])
end
