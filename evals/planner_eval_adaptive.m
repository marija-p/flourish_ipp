%clear all; close all; clc;

% If data already exists, want to append to it for the trials it contains.
append_to_logger = 0;

% Number of trials to run
if (~append_to_logger)
    num_trials = 3;
else
    trials = fieldnames(logger);
    trials = regexp(trials,'\d*','Match');
    trials = [trials{:}];
    trials_names = [];
    for i = 1:length(trials)
        trials_names = ...
            [trials_names; str2num(cell2mat(trials(i)))];
    end
end

% Environment parameters
max_cluster_radius = 3;
min_cluster_radius = 1;
dim_x_env = 30;
dim_y_env = 30;

[matlab_params, planning_params, ...
	opt_params, map_params] = load_params(dim_x_env, dim_y_env);
planning_params.time_budget = 200;  % [s]

%opt_methods = {'none', 'cmaes', 'fmc', 'bo'};
opt_methods = {'none', 'cmaes'};
%opt_methods = {};

use_rig = 0; subtree_iters = 500;
use_coverage = 1; coverage_altitude = 8.66; coverage_vel = 0.8;  %0.78 * (200/280)
use_random = 0;
use_greedy = 0; planning_params_greedy = planning_params;
planning_params_greedy.control_points = 2;

%logger = struct;

for i = 3:num_trials

    if (~append_to_logger)
        t = i;
    else
        t = trials_names(i);
    end

    logger.(['trial', num2str(t)]).num = t;
    
    % Generate (continuous) ground truth map.
    rng(t, 'twister');
    cluster_radius = randi([min_cluster_radius, max_cluster_radius]);
    interesting = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius, 0.6, 1);
    uninteresting = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius, 0, 0.4);
    ground_truth_map = interesting;
    ground_truth_map(:, (map_params.dim_y)/2+1:map_params.dim_y) = ...
        uninteresting(:, (map_params.dim_y)/2+1:map_params.dim_y);
    
    for j = 1:size(opt_methods,2)
        opt_params.opt_method = opt_methods{j};
        rng(t*j*planning_params.time_budget, 'twister');
        planning_params.use_threshold = 1;
        [metrics, grid_map_adaptive] = GP_iros(matlab_params, ...
            planning_params, opt_params, map_params, ground_truth_map);
        logger.(['trial', num2str(t)]).([opt_methods{j}, '_adaptive']) = ...
            metrics;
        planning_params.use_threshold = 0;
        [metrics, grid_map_nonadaptive] = GP_iros(matlab_params, ...
            planning_params, opt_params, map_params, ground_truth_map);
        logger.(['trial', num2str(t)]).([opt_methods{j}, '_nonadaptive']) = ...
            metrics;
        keyboard
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
    
    if (use_random)
        rng(t, 'twister');
        [metrics, ~] = GP_random(matlab_params, planning_params, ...
            map_params, ground_truth_map);
        logger.(['trial', num2str(t)]).('random') = metrics;
    end
    
    if (use_greedy)
        rng(t, 'twister')
        opt_params.opt_method = 'none';
        [metrics, ~] = GP_iros(matlab_params, ...
            planning_params_greedy, opt_params, map_params, ground_truth_map);
        logger.(['trial', num2str(t)]).('greedy') = metrics;
    end
    
    disp(['Completed Trial ', num2str(t)]);
    
end