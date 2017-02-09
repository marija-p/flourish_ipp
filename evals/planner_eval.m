clear all; close all; clc;

% Number of trials to run
num_trials = 1;

% Environment parameters
cluster_radius = 3;
dim_x_env = 30;
dim_y_env = 30;

[matlab_params, planning_params, ...
	opt_params, map_params] = load_params(dim_x_env, dim_y_env);

opt_methods = {'none', 'cmaes', 'fmc'};
%opt_methods = {'cmaes'};

logger = struct;

for t = 1:num_trials
    
    rng(matlab_params.seed_num, 'twister');
    
    % Generate (continuous) ground truth map.
    ground_truth_map = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius);
    
    for i = 1:size(opt_methods,2)
        opt_params.opt_method = opt_methods{i};
        [metrics, grid_map] = GP_iros(matlab_params, ...
            planning_params, opt_params, map_params, ground_truth_map);
        logger.(['trial', num2str(t)]).([opt_methods{i}]) = metrics;
        
    end
    
    logger.(['trial', num2str(t)]).num = t;
    
end