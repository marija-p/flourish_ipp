clear all; close all; clc;

% Random number generator
matlab_parameters.seed_num = 3;
rng(matlab_parameters.seed_num, 'twister');

%% Parameters %%

% Environment
cluster_radius = 3;
% Dimensions [m]
dim_x_env = 30;
dim_y_env = 30;

% Coefficients for the exponential height-dependant sensor variance model
% var = A * (1 - e^(-B * height))
planning_parameters.sensor_coeff_A = 0.05;
planning_parameters.sensor_coeff_B = 0.2;
% Camera fields of view (FoV)
planning_parameters.sensor_fov_angle_x = 60;
planning_parameters.sensor_fov_angle_y = 60;
planning_parameters.min_height = 1;
planning_parameters.max_height = 26;
planning_parameters.max_vel = 5;        % [m/s]
planning_parameters.max_acc = 3;        % [m/s^2]
planning_parameters.time_budget = 250;  % [s]

% Parameter to control exploration-exploitation trade-off in objective
planning_parameters.lambda = 0.001;

% Frequency at which to take measurements along a path [Hz]
planning_parameters.measurement_frequency = 0.1;

% Number of control points for a polynomial (start point fixed)
planning_parameters.control_points = 5;

% Threshold [% vegetation cover] - only regions above this value are
% considered "interesting" and used when computing information gain.
planning_parameters.lower_threshold = 0.3;
% Whether to use the threshold value for plannin
planning_parameters.use_threshold = 0;

optimization_parameters.max_iters = 25;
optimization_parameters.use_cmaes = 1;

% Map resolution [m/cell]
map_parameters.resolution = 0.75;
% Map dimensions [cells]
map_parameters.dim_x = dim_x_env/map_parameters.resolution;
map_parameters.dim_y = dim_y_env/map_parameters.resolution;
% Position of map in the environment [m]
map_parameters.position_x = -dim_x_env / 2;
map_parameters.position_y = -dim_y_env / 2;
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;
% Prediction map dimensions [cells]
predict_dim_x = dim_x*1;
predict_dim_y = dim_y*1;

matlab_parameters.visualize = 1;

% Gaussian Process
cov_func = {'covMaterniso', 3};
lik_func = @likGauss;
inf_func = @infExact;
mean_func = @meanConst;
% Hyperparameters
hyp.mean = 0.5;
hyp.cov =  [1.3 0.3];
hyp.lik =  0.35;

% First measurement location
point_init = [7.5, 7.5, 8.66];
% Multi-resolution lattice
lattice = create_lattice(map_parameters, planning_parameters, 25, 4);
 
%% Data %%
% Generate (continuous) ground truth map.
ground_truth_map = create_continuous_map(dim_x, dim_y, cluster_radius);
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,dim_x), linspace(1,dim_y,dim_y));
X_ref = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate prediction map.
[mesh_x,mesh_y] = meshgrid(linspace(1,predict_dim_x,predict_dim_x), ...
    linspace(1,predict_dim_y,predict_dim_y));
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate grid map.
grid_map.m = 0.5*ones(size(ground_truth_map));


%% Initial Measurement and Inference %%
% Generate prior map.
Y = reshape(grid_map.m,[],1);

% ymu, ys: mean and covariance for output
% fmu, fs: mean and covariance for latent variables
% post: struct representation of the (approximate) posterior
[ymu, ys, fmu, fs, ~ , post] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y, Z);
ymu = reshape(ymu, predict_dim_y, predict_dim_x);

alpha = post.alpha;
L = post.L; 
sW = post.sW;
Kss = real(feval(cov_func{:}, hyp.cov, Z));
Ks = feval(cov_func{:}, hyp.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
  V = L'\(sW.*Ks);
  grid_map.P = Kss - V'*V;                       % predictive variances
 else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  grid_map.P = Kss + Ks'*LKs;                    % predictive variances
end

%% This is just to test in case of no-inference using only the kernel
% sn2=exp(2*hyp.lik);
% K = feval(cov_func{:},hyp.cov,X_ref);
% KplusR = K+ sn2*eye(length(K));
% grid_map.P = KplusR;
%%

%grid_map = take_measurement_at_point(point_init, grid_map, ...
%    ground_truth_map, map_parameters, planning_parameters);
Y_sigma = sqrt(diag(grid_map.P)');
P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
P_trace_init = trace(grid_map.P);


%% Planning-Execution Loop %%
P_trace_prev = P_trace_init;
point_prev = point_init;

metrics = struct;
time_elapsed = 0;
metrics.path_travelled = [];
%metrics.points_meas = point_init;
%metrics.P_traces = trace(grid_map.P);
%metrics.times = 0;
metrics.points_meas = [];
metrics.P_traces = [];
metrics.times = [];
metrics.rmses = [];

while (time_elapsed < planning_parameters.time_budget)
    
    %% Planning %%
    
    %% STEP 1. Grid search on the lattice.
    path = search_lattice(point_prev, lattice, grid_map, map_parameters, ...
        planning_parameters);
    obj = compute_objective(path, grid_map, map_parameters, planning_parameters);
    disp(['Objective before optimization: ', num2str(obj)]);
    
    %% STEP 2. CMA-ES optimization.
    if (optimization_parameters.use_cmaes)
        path_optimized = optimize_with_cmaes(path, grid_map, map_parameters, ...
            planning_parameters, optimization_parameters);
            %obj = compute_objective(path_optimized, grid_map, map_parameters, planning_parameters);
            %disp(['Objective after optimization: ', num2str(obj)]);
    else
        path_optimized = path;
    end

    %% Plan Execution %%
    % Create polynomial trajectory through the control points.
    trajectory = ...
        plan_path_waypoints(path_optimized, ...
        planning_parameters.max_vel, planning_parameters.max_acc);

    % Sample trajectory to find locations to take measurements at.
    [times_meas, points_meas, ~, ~] = ...
        sample_trajectory(trajectory, 1/planning_parameters.measurement_frequency);

    % Take measurements along path, updating the grid map.
    for i = 1:size(points_meas,1)
        grid_map = take_measurement_at_point(points_meas(i,:), grid_map, ...
            ground_truth_map, map_parameters, planning_parameters);
        metrics.P_traces = [metrics.P_traces; trace(grid_map.P)];
        metrics.rmses = [metrics.rmses; compute_rmse(grid_map.m, ground_truth_map)];
    end

    Y_sigma = sqrt(diag(grid_map.P)');
    P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
    disp(['Trace after execution: ', num2str(trace(grid_map.P))]);
    disp(['Time after execution: ', num2str(get_trajectory_total_time(trajectory))]);
    gain = P_trace_prev - trace(grid_map.P);
    cost = get_trajectory_total_time(trajectory);
    disp(['Objective after execution: ', num2str(-gain*exp(-planning_parameters.lambda*cost))]);

    metrics.points_meas = [metrics.points_meas; points_meas];
    metrics.times = [metrics.times; time_elapsed + times_meas'];

    % Update variables for next planning stage.
    metrics.path_travelled = [metrics.path_travelled; path_optimized];
    P_trace_prev = trace(grid_map.P);
    
    point_prev = path_optimized(end,:); % End of trajectory (not last meas. point!)
    time_elapsed = time_elapsed + get_trajectory_total_time(trajectory);  
    
end


if (matlab_parameters.visualize)
    
    subplot(2, 1, 1)
    imagesc(grid_map.m)
    caxis([0, 1])
    title('Mean - final')
    set(gca,'Ydir','Normal');
    colorbar;
    
    subplot(2, 1, 2)
    contourf(P_post)
    title(['Var. Trace = ', num2str(trace(grid_map.P), 5)])
    set(gca,'Ydir','Normal');
    c = colorbar;
    P_climits = get(c, 'Limits');
    set(gcf, 'Position', [113, 279, 2402, 800]);
    
end