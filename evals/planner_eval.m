    %clear all; close all; clc;

% Number of trials to run
num_trials = 31;

% Environment parameters
max_cluster_radius = 3;
min_cluster_radius = 1;
dim_x_env = 30;
dim_y_env = 30;

[matlab_params, planning_params, ...
	opt_params, map_params] = load_params(dim_x_env, dim_y_env);
planning_params.time_budget = 600;  % [s]

%opt_methods = {'none', 'cmaes', 'fmc', 'bo'};
opt_methods = {};
use_rig = 1; subtree_iters = 500;
use_coverage = 0; coverage_altitude = 8.66; coverage_vel = 0.78;

%logger = struct;

for t = 2:num_trials
   
    logger.(['trial', num2str(t)]).num = t;
    
    % Generate (continuous) ground truth map.
    rng(t, 'twister');
    cluster_radius = randi([min_cluster_radius, max_cluster_radius]);
    ground_truth_map = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius);
    
    for i = 1:size(opt_methods,2)
        opt_params.opt_method = opt_methods{i};
        rng(t*i, 'twister');
        [metrics, ~] = GP_iros(matlab_params, ...
            planning_params, opt_params, map_params, ground_truth_map);
        logger.(['trial', num2str(t)]).([opt_methods{i}]) = metrics;
    end
    
    if (use_rig)
        rng(t, 'twister');
        [metrics, ~] = GP_rig(matlab_params, ...
            planning_params, map_params, subtree_iters, ground_truth_map);
        logger.(['trial', num2str(t)]).('rig') = metrics;
    end
    
    if (use_coverage)
        rng(t, 'twister');
        [metrics, ~] = GP_coverage(matlab_params, ...
            planning_params, map_params, coverage_altitude, coverage_vel, ground_truth_map);
        logger.(['trial', num2str(t)]).('coverage') = metrics;
    end
    
    disp(['Completed Trial', num2str(t)]);
    
end