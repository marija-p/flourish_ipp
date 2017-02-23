 dim_x_env = 2;
 dim_y_env = 2;
 cluster_radius = 3;
 
 % Define static transformations required for co-ordinate conversions.
transforms.T_VSB_CAM = eye(4);  % Vicon-sensor-body -> camera
transforms.T_MAP_W = eye(4);    % Map -> world

 [matlab_parameters, planning_parameters, optimization_parameters, ...
     map_parameters] = load_params_ros(dim_x_env, dim_y_env);
 
 [metrics, grid_map] = GP_iros_ros(planning_parameters, optimization_parameters, ...
    map_parameters, transforms);