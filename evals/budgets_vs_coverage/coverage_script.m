%clear all; close all; clc;


% Fixed lawnmower coverage altitude
%coverage_altitude = 12.99;
%coverage_altitude = 6.5;
coverage_altitude = 8.66;
%coverage_altitude = 5.2;
    
% Environment parameters
cluster_radius = 3;
dim_x_env = 30;
dim_y_env = 30;

[matlab_params, planning_params, ...
	opt_params, map_params] = load_params(dim_x_env, dim_y_env);

% Initialize velocity.
planning_params.max_vel = 0.32;

rng(matlab_params.seed_num, 'twister');

% Generate (continuous) ground truth map.
ground_truth_map = create_continuous_map(map_params.dim_x, ...
    map_params.dim_y, cluster_radius, 0, 1);

disp(['Velocity: ', num2str(planning_params.max_vel)]);
[metrics, grid_map] = GP_coverage(matlab_params, ...
    planning_params, map_params, coverage_altitude, ...
    planning_params.max_vel, ground_truth_map);

disp(['Final Tr(P): ', num2str(metrics.P_traces(end))]);

%close all;