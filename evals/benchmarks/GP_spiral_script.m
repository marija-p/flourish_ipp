%clear all; close all; clc;

% Number of trials to run
num_trials = 30;

% Spiral parameters
spiral_height = 12.5;

% Environment parameters
cluster_radius = 3;
dim_x_env = 30;
dim_y_env = 30;

[matlab_params, planning_params, ...
	opt_params, map_params] = load_params(dim_x_env, dim_y_env);

logger = struct;

for t = 1:num_trials
    
    logger.(['trial', num2str(t)]).num = t;
    
    rng(t, 'twister');
    
    % Generate (continuous) ground truth map.
    ground_truth_map = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius);
    planning_params.max_vel = 0.9;
    planning_params.max_acc = 3;
    
    [metrics, grid_map] = GP_spiral(matlab_params, ...
        planning_params, map_params, spiral_height, ground_truth_map);
    logger.(['trial', num2str(t)]).('spiral') = metrics;
    
    %keyboard
    
end

save spiral.mat