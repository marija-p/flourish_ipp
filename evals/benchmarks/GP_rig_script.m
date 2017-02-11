clear all; close all; clc;

% Number of trials to run
num_trials = 1;

% Environment parameters
cluster_radius = 3;
dim_x_env = 30;
dim_y_env = 30;

[matlab_params, planning_params, ...
	opt_params, map_params] = load_params(dim_x_env, dim_y_env);

logger = struct;

for t = 1:num_trials
    
    rng(matlab_params.seed_num, 'twister');
    
    % Generate (continuous) ground truth map.
    ground_truth_map = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius);
    
    [metrics, grid_map] = GP_rig(matlab_params, ...
        planning_params, map_params, ground_truth_map);
    logger.(['trial', num2str(t)]).('rig') = metrics;
    logger.(['trial', num2str(t)]).num = t;
    
end