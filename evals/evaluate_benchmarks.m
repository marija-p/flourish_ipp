%clear all; close all; clc;

% If data already exists, want to append to it for the trials it contains.
append_to_logger = 0;

% Number of trials to run
if (~append_to_logger)
    num_trials = 30;
else
    trials = fieldnames(logger);
    trials = regexp(trials,'\d*','Match');
    trials = [trials{:}];
    trials_names = [];
    for i = 1:length(trials)
        trials_names = ...
            [trials_names; str2num(cell2mat(trials(i)))];
    end
    num_trials = size(trials_names,1);
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
opt_methods = {};
use_rig = 1; subtree_iters = 500;
use_coverage = 1; coverage_altitude = 8.66; coverage_vel = 0.78 * (200/200);

%logger = struct;

for i = 1:num_trials

    if (~append_to_logger)
        t = i;
    else
        t = trials_names(i);
    end
    
    logger.(['trial', num2str(t)]).num = t;
    
    % Generate (continuous) ground truth map.
    rng(t, 'twister');
    cluster_radius = randi([min_cluster_radius, max_cluster_radius]);
    ground_truth_map = create_continuous_map(map_params.dim_x, ...
        map_params.dim_y, cluster_radius);
    
    try
        
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
        
        disp(['Completed Trial ', num2str(t)]);
        
    catch
        
        disp(['Failed Trial ', num2str(t)]);
        
    end
    
    save rig_coverage.mat
    
end