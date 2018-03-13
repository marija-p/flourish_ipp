function [metrics, grid_map] = ...
    GP_iros(matlab_params, planning_params, ...
    opt_params, map_params, ground_truth_map)
% Main program for IROS2017 IPP algorithms.
%
% M Popovic 2017
%

% Get map dimensions [cells]
dim_x = map_params.dim_x;
dim_y = map_params.dim_y;
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
% Multi-resolution lattice
lattice = create_lattice(map_params, planning_params);
 
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
sn2=exp(2*hyp.lik);
K = feval(cov_func{:},hyp.cov,X_ref);
KplusR = K+ sn2*eye(length(K));
KplusR_inv = eye(size(K))/KplusR ;
Kss = feval(cov_func{:},hyp.cov, Z) ;
Kst = feval(cov_func{:},hyp.cov, Z, X_ref) ;
grid_map.PP = Kss - Kst*KplusR_inv*Kst';


%grid_map = take_measurement_at_point(point_init, grid_map, ...
%    ground_truth_map, map_parameters, planning_parameters);
Y_sigma = sqrt(diag(grid_map.P)');
P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
P_trace_init = trace(grid_map.P);


%% Planning-Execution Loop %%
P_trace_prev = P_trace_init;
point_prev = point_init;
time_elapsed = 0;
budget_spent = 0;
metrics = initialize_metrics();

while (true)
    
    %% Planning %%
    
    %% STEP 1. Grid search on the lattice.
    tic;
    
    path = search_lattice(point_prev, lattice, grid_map, map_params, ...
        planning_params);
    obj = compute_objective(path, grid_map, map_params, planning_params);
    %disp(['Objective before optimization: ', num2str(obj)]);
    
    %disp(toc);
    %keyboard
    
    %% STEP 2. Path optimization.
    if (strcmp(opt_params.opt_method, 'cmaes'))
        path_optimized = optimize_with_cmaes(path, grid_map, map_params, ...
            planning_params, opt_params);
            %obj = compute_objective(path_optimized, grid_map, map_parameters, planning_parameters);
            %disp(['Objective after optimization: ', num2str(obj)]);
    elseif (strcmp(opt_params.opt_method, 'fmc'))
        path_optimized = optimize_with_fmc(path, grid_map, map_params, ...
            planning_params);
    elseif (strcmp(opt_params.opt_method, 'bo'))
        path_optimized = optimize_with_bo(path, grid_map, map_params, ...
            planning_params);
    elseif (strcmp(opt_params.opt_method, 'sa'))
        path_optimized = optimize_with_sa(path, grid_map, map_params, ...
            planning_params);
    else
        path_optimized = path;
    end
    
    %% Plan Execution %%
    % Create polynomial trajectory through the control points.
    trajectory = ...
        plan_path_waypoints(path_optimized, ...
        planning_params.max_vel, planning_params.max_acc);

    % Sample trajectory to find locations to take measurements at.
    [times_meas, points_meas, ~, ~] = ...
        sample_trajectory(trajectory, 1/planning_params.measurement_frequency);
    
    % Take measurements along path, updating the grid map.
    for i = 1:size(points_meas,1)
        
        % Budget has been spent.
        if ((time_elapsed + times_meas(i)) > planning_params.time_budget)
            points_meas = points_meas(1:i-1,:);
            times_meas = times_meas(1:i-1);
            budget_spent = 1;
            break;
        end
        
        grid_map = take_measurement_at_point(points_meas(i,:), grid_map, ...
            ground_truth_map, map_params, planning_params);
        metrics.P_traces = [metrics.P_traces; trace(grid_map.P)];
        metrics.rmses = [metrics.rmses; compute_rmse(grid_map.m, ground_truth_map)];
        metrics.wrmses = [metrics.wrmses; compute_wrmse(grid_map.m, ground_truth_map)];
        metrics.mlls = [metrics.mlls; compute_mll(grid_map, ground_truth_map)];
        metrics.wmlls = [metrics.wmlls; compute_wmll(grid_map, ground_truth_map)];
        
    end

    Y_sigma = sqrt(diag(grid_map.P)');
    P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
    %disp(['Trace after execution: ', num2str(trace(grid_map.P))]);
    %disp(['Time after execution: ', num2str(get_trajectory_total_time(trajectory))]);
    gain = P_trace_prev - trace(grid_map.P);
    if (strcmp(planning_params.obj, 'rate'))
        cost = max(get_trajectory_total_time(trajectory), 1/planning_params.measurement_frequency);
        %disp(['Objective after execution: ', num2str(-gain/cost)]);
    elseif (strcmp(planning_params.obj, 'exponential'))
        cost = get_trajectory_total_time(trajectory);
        %disp(['Objective after execution: ', num2str(-gain*exp(-planning_params.lambda*cost))]);
    end
    
    metrics.points_meas = [metrics.points_meas; points_meas];
    metrics.times = [metrics.times; time_elapsed + times_meas'];

    % Update variables for next planning stage.
    metrics.path_travelled = [metrics.path_travelled; path_optimized];
    P_trace_prev = trace(grid_map.P);
    
    point_prev = path_optimized(end,:); % End of trajectory (not last meas. point!)
    time_elapsed = time_elapsed + get_trajectory_total_time(trajectory);  

    if (budget_spent)
        break;
    end
    
end

if (matlab_params.visualize)
    
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
    set(gcf, 'Position', [113, 279, 2402, 800]);
    
end