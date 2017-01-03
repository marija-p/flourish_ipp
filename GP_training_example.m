clear all; close all; clc;

%% Parameters %%

% Environment
cluster_radius = 3;
% Dimensions [m]
dim_x_env = 10;
dim_y_env = 10;

% Map resolution [m/cell]
map_parameters.resolution = 0.5;
% Map dimensions [cells]
map_parameters.dim_x = dim_x_env/map_parameters.resolution;
map_parameters.dim_y = dim_y_env/map_parameters.resolution;
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;

matlab_parameters.visualize = 0;

% Gaussian Process
cov_func = {'covMaterniso', 3};
lik_func = @likGauss;
inf_func = @infExact;
mean_func = @meanConst;
% Hyperparameters
hyp.mean = 0.5;
hyp.cov = [-1,-0.76];   % With low correlation
hyp.lik = -0.7;         % Roughly covers from 0 to 1 in 2*sigma bounds


%% Data %%

% Generate (continuous) ground truth map.
ground_truth_map = create_continuous_map(dim_x, dim_y, cluster_radius);
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,dim_x), linspace(1,dim_y,dim_y));
X_ref = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];
Y_ref = reshape(ground_truth_map,[],1);


%% Training %%
% Optimise hyperparameters.
% Number of conjugate gradient steps
N_cg = -100;
[hyp, ~, ~] = ...
    minimize(hyp, 'gp', N_cg, inf_func, mean_func, cov_func, lik_func, X_ref, Y_ref);
disp(hyp)


%% Plotting %%
if (matlab_parameters.visualize)
    
    % Means
    figure;
    subplot(1,3,1)
    imagesc(ground_truth_map)
    caxis([0, 1])
    title('Ground truth map')
    set(gca,'Ydir', 'Normal');
    
    subplot(1,3,2)
    imagesc(ymu)
    caxis([0, 1])
    title('Mean - prior (ymu)')
    set(gca,'Ydir','Normal');
    
    subplot(1,3,3)
    imagesc(grid_map_post.m)
    caxis([0, 1])
    title('Mean - posterior (grid\_map\_post)')
    set(gca,'Ydir','Normal');
    c1 = colorbar;
    set(gcf, 'Position', [58, 328, 1863, 485]);
    
    % Variances
    figure;
    subplot(1,2,1)
    contourf(P_prior)
    title('Prior variance')
    set(gca,'Ydir','Normal');
    
    subplot(1,2,2)
    contourf(P_post)
    title('Posterior variance')
    set(gca,'Ydir','Normal');
    c2 = colorbar;
    set(gcf, 'Position', [752, 615, 1001, 405])
    
end