clear all; close all; clc;

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

% Number of points in global plan over a horizon
optimization_parameters.planning_horizon_waypoints = 6;
optimization_parameters.max_iters = 15;

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
visualize_path = 1;

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
pos_env_init = [0, 0, 6];
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


%% Measurement and Inference %%
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

% Extract variance map (diagonal elements).
Y_sigma = sqrt(diag(grid_map.P)');
P_prior = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);

if (matlab_parameters.visualize)
    
    figure;
    subplot(2, 4, 1)
    imagesc(ground_truth_map)
    caxis([0, 1])
    title('Ground truth map')
    set(gca,'Ydir', 'Normal');
    
    subplot(2, 4, 2)
    imagesc(ymu)
    caxis([0, 1])
    title('Mean - prior')
    set(gca,'Ydir', 'Normal');
    
    subplot(2, 4, 6)
    contourf(P_prior)
    title(['Var. Trace = ', num2str(trace(grid_map.P), 5)])
    set(gca,'Ydir','Normal');
 
end

% Take an initial measurement.
grid_map = take_measurement_at_point(pos_env_init, grid_map, ...
    ground_truth_map, map_parameters, planning_parameters);
Y_sigma = sqrt(diag(grid_map.P)');
P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
P_trace_init = trace(grid_map.P);
    
if (matlab_parameters.visualize)

    subplot(2, 4, 3)
    imagesc(grid_map.m)
    caxis([0, 1])
    title('Mean - init.')
    set(gca,'Ydir', 'Normal');
    
    subplot(2, 4, 7)
    contourf(P_post)
    title(['Var. Trace = ', num2str(trace(grid_map.P), 5)])
    set(gca,'Ydir','Normal');
    
end


%% Planning %%

%% STEP 1. Grid search on the lattice.
P_trace_prev = P_trace_init;
pos_env_prev = pos_env_init;
grid_map_plan = grid_map;
path = pos_env_init;
% Path travel distance
path_length = 0;

while (optimization_parameters.planning_horizon_waypoints > size(path, 1))
    
    % Initialise best solution so far.
    obj_min = Inf;
    pos_env_best = -Inf;

    for i = 1:size(lattice, 1)

        pos_env_eval = lattice(i, :);
        grid_map_eval = predict_map_update(pos_env_eval, grid_map_plan, ...
            map_parameters, planning_parameters);
        P_trace = trace(grid_map_eval.P);
        
        gain = P_trace_prev - P_trace;
        % Limit the min. distance between measurements.
        cost = max(pdist([pos_env_prev; pos_env_eval]), 5);
        obj = -gain/cost;

        %disp(['Evaluating Candidate No. ', num2str(i), ': ', num2str(pos_env_eval)]);
        %disp(['Trace of P: ', num2str(trace(grid_map_eval.P))]);
        
        % Update best solution.
        if (obj < obj_min)
            obj_min = obj;
            pos_env_best = pos_env_eval;
        end
        
    end
    
    grid_map_plan = predict_map_update(pos_env_best, grid_map_plan, ...
        map_parameters, planning_parameters);
    disp(['Point ', num2str(size(path,1)+1), ' at: ', num2str(pos_env_best)]);
    disp(['Trace of P: ', num2str(trace(grid_map_plan.P))]);
    path = [path; pos_env_best];
    path_length = path_length + max(pdist(path(end-1:end,:)), 5);
    
    P_trace_prev = trace(grid_map_plan.P);
    pos_env_prev = pos_env_best;
    
end
disp(['Trace before optimization: ', num2str(trace(grid_map_plan.P))]);
disp(['Distance before optimization: ', num2str(path_length)]);
disp(['Objective after optimization: ', ...
    num2str((P_trace_init - trace(grid_map_plan.P)) / path_length)]);

%% STEP 2. CMA-ES optimization.
% Set optimization parameters.
opt = cmaes;
opt.DispFinal = 'off';
opt.LogModulo = 0;
opt.TolFun = 1e-9;
opt.IncPopSize = 1;
opt.SaveVariables = 'off';
opt.MaxIter = optimization_parameters.max_iters;
opt.Seed = randi(2^10);

% Set bounds and covariances.
LBounds = [-dim_x_env/2;-dim_y_env/2;planning_parameters.min_height];
UBounds = [dim_x_env/2;dim_y_env/2;planning_parameters.max_height];
opt.LBounds = repmat(LBounds, size(path,1)-1, 1);
opt.UBounds = repmat(UBounds, size(path,1)-1, 1);
cov = [10; 10; 5];
cov = repmat(cov, size(path,1)-1, 1);

% Remove starting point (as this is fixed).
starting_point = path(1,:);
path = reshape(path(2:end,:)', [], 1);
path_optimized = cmaes('optimize_points', path, cov, opt, starting_point, grid_map, ...
    map_parameters, planning_parameters);
path_optimized = reshape(path_optimized, 3, [])';
path_optimized = [starting_point; path_optimized];


%% Measurement
path_length = 0;
for i = 2:size(path_optimized,1)
    
    grid_map = take_measurement_at_point(path_optimized(i,:), grid_map, ...
        ground_truth_map, map_parameters, planning_parameters);
    path_length = path_length + max(pdist(path_optimized(i-1:i,:)),5);
    
end

Y_sigma = sqrt(diag(grid_map.P)');
P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
disp(['Trace after optimization: ', num2str(trace(grid_map.P))]);
disp(['Distance after optimization: ', num2str(path_length)]);
disp(['Objective after optimization: ', ...
    num2str((P_trace_init - trace(grid_map.P)) / path_length)]);

if (matlab_parameters.visualize)
    
    subplot(2, 4, 4)
    imagesc(grid_map.m)
    caxis([0, 1])
    title('Mean - final')
    set(gca,'Ydir','Normal');
    colorbar;
    
    subplot(2, 4, 8)
    contourf(P_post)
    title(['Var. Trace = ', num2str(trace(grid_map.P), 5)])
    set(gca,'Ydir','Normal');
    c = colorbar;
    P_climits = get(c, 'Limits');
    set(gcf, 'Position', [113, 279, 2402, 800]);
    
    % Scale variance plot colours.
    subplot(2, 4, 6)
    caxis(P_climits)
    subplot(2, 4, 7)
    caxis(P_climits)
    
end

if (visualize_path)
    
    figure;
    plot_path(path_optimized);
    
end