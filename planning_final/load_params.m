function [matlab_params, planning_params, ...
    opt_params, map_params] = load_params(dim_x_env, dim_y_env, visualize)
% Loads default parameters for IROS2017 IPP algorithms - AURO19.

% Random number generator
matlab_params.seed_num = 2;
if ~exist('visualize','var')
    matlab_params.visualize = 0;
else
    matlab_params.visualize = visualize;
end

% Coefficients for the exponential height-dependant sensor variance model
% var = A * (1 - e^(-B * height))
planning_params.sensor_coeff_A = 0.05;
planning_params.sensor_coeff_B = 0.2;
% Camera fields of view (FoV)
planning_params.sensor_fov_angle_x = 60;
planning_params.sensor_fov_angle_y = 60;
planning_params.min_height = 1;
planning_params.max_height = 26;
planning_params.max_vel = 5;        % [m/s]
planning_params.max_acc = 3;        % [m/s^2]
planning_params.time_budget = 200;  % [s]

% Parameter to control exploration-exploitation trade-off in objective
planning_params.lambda = 0.01;

% Frequency at which to take measurements along a path [Hz]
planning_params.measurement_frequency = 0.15;

% Number of control points for a polynomial (start point fixed)
planning_params.control_points = 5;

% Number of lattice points at lowest altitude level
planning_params.lattice_min_height_points = 16;
% Distance between successive altitude levels on the lattice
planning_params.lattice_height_increment = 8;

% Threshold [% vegetation cover] - only regions above this value are
% considered "interesting" and used when computing information gain.
planning_params.lower_threshold = 0.4;
% Whether to use the threshold value for active planning
planning_params.use_threshold = 1;

% Objective function for planning
planning_params.obj = 'rate';    % 'rate'/'exponential'

opt_params.max_iters = 45;
opt_params.opt_method = 'none'; % 'fmc'/cmaes'/'none'/'bo'
% Covariances in each search dimension
opt_params.cov_x = 3;
opt_params.cov_y = 3;
opt_params.cov_z = 4;

% Map resolution [m/cell]
map_params.resolution = 0.75;
% Map dimensions [cells]
map_params.dim_x = dim_x_env/map_params.resolution;
map_params.dim_y = dim_y_env/map_params.resolution;
% Position of map in the environment [m]
map_params.position_x = -dim_x_env / 2;
map_params.position_y = -dim_y_env / 2;
% if false, map lattice is dynamically created dependent on map dimensions
map_params.manual_lattice = 0;