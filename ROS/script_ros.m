 dim_x_env = 2;
 dim_y_env = 2;
 
 % Define static transformations required for co-ordinate conversions.
transforms.T_VSB_CAM = ...              % Vicon sensor body -> camera.
    [0, -1.0000, 0, -0.0140;
    -1.0000,    0,    0,   -0.0000;
     0,    0,   -1.0000,   -0.0559;
     0,         0,         0,    1.0000];
transforms.T_MAP_W = eye(4);    % Map -> world

 [matlab_parameters, planning_parameters, optimization_parameters, ...
     map_parameters] = load_params_ros(dim_x_env, dim_y_env);
 
 [metrics, grid_map] = GP_iros_ros(planning_parameters, optimization_parameters, ...
    map_parameters, transforms);