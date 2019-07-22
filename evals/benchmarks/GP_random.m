function [metrics, grid_map] = GP_random(~, planning_params, ...
    map_params, ground_truth_map)
% Main program for JFR18 random algorithm benchmark.
%
% M Popovic 2018
%

% Initialize variables.

%matlab_parameters.visualize = 1;
planning_params = rmfield(planning_params, 'control_points');   % For plotting

% Get map dimensions [cells]
dim_x = map_params.dim_x;
dim_y = map_params.dim_y;
% Get map dimensions [m]
dim_x_env = dim_x*map_params.resolution;
dim_y_env = dim_y*map_params.resolution;
% Set prediction map dimensions [cells]
predict_dim_x = dim_x*1;
predict_dim_y = dim_y*1;

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
 
%% Data %%
% Generate (continuous) ground truth map.
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

budget_spent = 0;
time_elapsed = 0;
metrics = initialize_metrics();

point_prev = point_init;

%% Planning-Execution Loop %%
while (true)
    
    % keyboard
    
    % Sample a random location in the environment to go to.
    x_rand = dim_x_env*rand(1,1) - dim_x_env/2;
    y_rand = dim_y_env*rand(1,1) - dim_y_env/2;
    z_rand = ...
        (planning_params.max_height-planning_params.min_height)*rand(1,1) + ...
        planning_params.min_height;
    
    point_next = [x_rand, y_rand, z_rand];
    
    % Connect current position to next point.
    path_current = [point_prev; point_next];

    % Create polynomial trajectory through the control points.
    trajectory = ...
        plan_path_waypoints(path_current, ...
        planning_params.max_vel, planning_params.max_acc);
    
    % Sample trajectory to find locations to take measurements at.
    [times_meas, points_meas, ~, ~] = ...
        sample_trajectory(trajectory, 1/planning_params.measurement_frequency);

    % Update metrics for the planned path.
    % Take measurements along path, updating the grid map.
    for i = 1:size(points_meas,1)

        % Budget has been spent.
        if ((time_elapsed + times_meas(i)) > planning_params.time_budget)
            points_meas = points_meas(1:i-1,:);
            times_meas = times_meas(1:i-1);
            budget_spent = 1;
            break;
        end

        %disp(['Taking measurement at: ', num2str(points_meas(i,:))]);
        
        grid_map = take_measurement_at_point(points_meas(i,:), grid_map, ...
            ground_truth_map, map_params, planning_params);
        metrics.P_traces = [metrics.P_traces; trace(grid_map.P)];
        metrics.rmses = [metrics.rmses; compute_rmse(grid_map.m, ground_truth_map)];
        metrics.wrmses = [metrics.wrmses; compute_wrmse(grid_map.m, ground_truth_map)];
        metrics.mlls = [metrics.mlls; compute_mll(grid_map, ground_truth_map)];
        metrics.wmlls = [metrics.wmlls; compute_wmll(grid_map, ground_truth_map)];
        
    end

    metrics.points_meas = [metrics.points_meas; points_meas];
    metrics.times = [metrics.times; time_elapsed + times_meas'];
    metrics.path_travelled = [metrics.path_travelled; path_current];
    
    time_elapsed = time_elapsed + get_trajectory_total_time(trajectory);  

    %disp(['Elapsed time: ', num2str(time_elapsed)]);
    
    if (budget_spent)
        break;
    end
    
    % Assuming UAV stops at the end of the trajectory.
    point_prev = point_next;
    
end

end