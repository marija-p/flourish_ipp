function [matlab_params, planning_params, ...
    opt_params, map_params] = load_params_iw(dim_x_env, dim_y_env)
% Loads default parameters for IROS2017 IPP algorithms.

% Random number generator
matlab_params.seed_num = 2;
matlab_params.visualize = 0;

% Coefficients for the exponential height-dependant sensor variance model
% var = A * (1 - e^(-B * height))
planning_params.sensor_coeff_A = 0.06;
planning_params.sensor_coeff_B = 0.2;
% Camera fields of view (FoV)
planning_params.sensor_fov_angle_x = 42.7;
planning_params.sensor_fov_angle_y = 55;
planning_params.min_height = 2;
planning_params.max_height = 7.6;
planning_params.max_vel = 1.5;        % [m/s]
planning_params.max_acc = 2;          % [m/s^2]
planning_params.time_budget = 1500;   % [s]

% Parameter to control exploration-exploitation trade-off in objective
planning_params.lambda = 0.01;

% Frequency at which to take measurements along a path [Hz]
planning_params.measurement_frequency = 0.2;

% Number of control points for a polynomial (start point fixed)
planning_params.control_points = 5;

% Number of lattice points at lowest altitude level
planning_params.lattice_min_height_points = 9;
% Distance between successive altitude levels on the lattice
planning_params.lattice_height_increment = 2.8;

% Threshold [% vegetation cover] - only regions above this value are
% considered "interesting" and used when computing information gain.
planning_params.lower_threshold = 0.15;
% Whether to use the threshold value for active planning
planning_params.use_threshold = 1;

% Objective function for planning
planning_params.obj = 'rate';    % 'rate'/'exponential'

opt_params.max_iters = 15;
opt_params.opt_method = 'cmaes'; % 'fmc'/cmaes'/'none'/'bo'
% Covariances in each search dimension
opt_params.cov_x = 0.2;
opt_params.cov_y = 0.2;
opt_params.cov_z = 0.2;

% Map resolution [m/cell]
map_params.resolution = 0.2;
% Map dimensions [cells]
map_params.dim_x = dim_x_env/map_params.resolution;
map_params.dim_y = dim_y_env/map_params.resolution;
% Position of map in the environment [m]
map_params.position_x = -dim_x_env / 2;
map_params.position_y = -dim_y_env / 2;