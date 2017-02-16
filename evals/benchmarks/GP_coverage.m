function [metrics, grid_map] = GP_coverage(matlab_parameters, planning_parameters, ...
    map_parameters, coverage_altitude, coverage_vel, ground_truth_map)
% Main program for IROS2017 coverage planning benchmark.
%
% M Popovic 2017
%

% Initialize variables.

matlab_parameters.visualize = 1;

% Get map dimensions [cells].
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;
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

%% This is just to test in case of no-inference using only the kernel
% sn2=exp(2*hyp.lik);
% K = feval(cov_func{:},hyp.cov,X_ref);
% KplusR = K+ sn2*eye(length(K));
% grid_map.P = KplusR;
%%

%% Coverage Planning %%

% Set starting point.
submap_edge_size = get_submap_edge_size(coverage_altitude, ...
    map_parameters, planning_parameters);
point = [submap_edge_size.y/2, submap_edge_size.x/2, coverage_altitude];

% Create plan (deterministic).
path = point;
i = 0;

while (path(end,1) < map_parameters.dim_x)
   
    % Move in y.
    if (mod(i,2) == 0)
        point = path(end,:) + [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];
        point = path(end,:) + [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];
        point = path(end,:) + [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];
        point = path(end,:) + [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];
    else
        point = path(end,:) - [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];
        point = path(end,:) - [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];
        point = path(end,:) - [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];
        point = path(end,:) - [0, (map_parameters.dim_y-submap_edge_size.y)/4, 0];
        path = [path; point];     
    end
    
    % Move in x.
    point = path(end,:) + [submap_edge_size.x, 0, 0];
    path = [path; point];
    i = i + 1;
    
end

% Remove the last waypoint (out of bounds).
path = path(1:end-1, :);
path = grid_to_env_coordinates(path, map_parameters);



%% Execution %%
metrics = initialize_metrics();

% Create polynomial trajectory through the control points.
trajectory = ...
    plan_path_waypoints(path, ...
   coverage_vel, planning_parameters.max_acc);

% Sample trajectory to find locations to take measurements at.
[times_meas, points_meas, ~, ~] = ...
    sample_trajectory(trajectory, 1/planning_parameters.measurement_frequency);

% Take measurements along path, updating the grid map.
for i = 1:size(points_meas,1)
    grid_map = take_measurement_at_point(points_meas(i,:), grid_map, ...
        ground_truth_map, map_parameters, planning_parameters);
    metrics.P_traces = [metrics.P_traces; trace(grid_map.P)];
    metrics.rmses = [metrics.rmses; compute_rmse(grid_map.m, ground_truth_map)];
    metrics.wrmses = [metrics.wrmses; compute_wrmse(grid_map.m, ground_truth_map)];
    metrics.mlls = [metrics.mlls; compute_mll(grid_map, ground_truth_map)];
    metrics.wmlls = [metrics.wmlls; compute_wmll(grid_map, ground_truth_map)];
end

Y_sigma = sqrt(diag(grid_map.P)');
P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);

metrics.points_meas = points_meas;
metrics.times = times_meas';
metrics.path_travelled = path;
disp(['Total time taken for coverage: ', ...
    num2str(get_trajectory_total_time(trajectory))]);
disp(['Total number of measurements: ', num2str(size(points_meas, 1))]);

if (matlab_parameters.visualize)
    
    subplot(1, 2, 1)
    imagesc(grid_map.m)
    caxis([0, 1])
    title('Mean - final')
    set(gca,'Ydir','Normal');
    colorbar;
    
    subplot(1, 2, 2)
    contourf(P_post)
    title(['Var. Trace = ', num2str(trace(grid_map.P), 5)])
    set(gca,'Ydir','Normal');
    c = colorbar;
    P_climits = get(c, 'Limits');
    set(gcf, 'Position', [1000, 1014, 863, 296]);
    
end