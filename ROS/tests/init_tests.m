 dim_x_env = 2;
 dim_y_env = 2;
 
 [matlab_parameters, planning_parameters, optimization_parameters, ...
     map_parameters] = load_params_ros(dim_x_env, dim_y_env);
 grid_map = create_empty_map(map_parameters);