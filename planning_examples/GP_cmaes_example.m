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
visualize_lattice = 1;
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
% Number of measurements to take (including initial one)
num_of_measurements = 20;
 
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


%% Candidate Evalaution (Planning ;)) %%
%lattice = lattice(1:50, :);
P_trace_prev = P_trace_init;
pos_env_prev = pos_env_init;

path = [];

for k = 1:num_of_measurements-1
    
    % Initialise best solution so far.
    obj_max = -Inf;
    pos_env_best = -Inf;
    obj_lattice = zeros(size(lattice,1), num_of_measurements-1);

    for i = 1:size(lattice, 1)

        pos_env_eval = lattice(i, :);
        grid_map_eval = take_measurement_at_point(pos_env_eval, grid_map, ...
            ground_truth_map, map_parameters, planning_parameters);
        P_trace = trace(grid_map_eval.P);
        
        gain = P_trace_prev - P_trace;
        % Limit the min. distance between measurements.
        cost = max(pdist([pos_env_prev; pos_env_eval]), 5);
        obj = gain/cost;
        obj_lattice(i, k) = obj;

        %disp(['Evaluating Candidate No. ', num2str(i), ': ', num2str(pos_env_eval)]);
        %disp(['Trace of P: ', num2str(trace(grid_map_eval.P))]);

        % Update best solution.
        if (obj > obj_max)
            obj_max = obj;
            pos_env_best = pos_env_eval;
        end

    end

    % Take final measurement at best point.
    grid_map = take_measurement_at_point(pos_env_best, grid_map, ...
        ground_truth_map, map_parameters, planning_parameters);
    Y_sigma = sqrt(diag(grid_map.P)');
    P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
    disp(['Taking Measurement ', num2str(k + 1), ' at: ', num2str(pos_env_best)]);
    disp(['Trace of P: ', num2str(trace(grid_map.P))]);
    
    P_trace_prev = trace(grid_map.P);
    pos_env_prev = pos_env_best;
    
    path = [path; pos_env_best];

end

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

% Plot the (final) informative objective values on the lattice.
if (visualize_lattice)
    
    figure;
    scatter3(lattice(:,1), lattice(:,2), lattice(:,3), 46, ...
      obj_lattice(:, k), 'filled');
    c = colorbar;
    ylabel(c, 'Info. value')
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    title('Lattice info. evaluation')
    grid minor
    
end

if (visualize_path)
    
    figure;
    plot_path(path);
    
end