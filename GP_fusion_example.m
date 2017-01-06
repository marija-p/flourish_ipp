clear all; close all; clc;

%% Parameters %%

% Environment
cluster_radius = 3;
% Dimensions [m]
dim_x_env = 10;
dim_y_env = 10;

% Camera fields of view (FoV)
planning_parameters.sensor_fov_angle_x = 60;
planning_parameters.sensor_fov_angle_y = 60;

% Map resolution [m/cell]
map_parameters.resolution = 0.5;
% Map dimensions [cells]
map_parameters.dim_x = dim_x_env/map_parameters.resolution;
map_parameters.dim_y = dim_y_env/map_parameters.resolution;
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;
% Prediction map dimensions [cells]
predict_dim_x = dim_x*1;
predict_dim_y = dim_y*1;

matlab_parameters.visualize = 1;

% Gaussian Process
%cov_func = {'covSEiso'};
cov_func = {'covMaterniso', 3};
lik_func = @likGauss;
inf_func = @infExact;
mean_func = @meanConst;

% Hyperparameters
hyp.mean = 0.5;
hyp.cov = [0.01 0.5];
hyp.lik = -0.5;

%hyp.cov =  [0.01 0.5];%[1, -0.76];
%cov = [0.5, 2];
%hyp.cov = log(cov);
%hyp.lik = -0.5; %-0.3; %-0.3


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
grid_map_prior.m = 0.5*ones(size(ground_truth_map));
%grid_map_prior.m = ground_truth_map;

%% Training %%
% Optimise hyperparameters.
% *** TODO (M): Not sure how to do this at the moment. ***
% Number of conjugate gradient steps
%N_cg = -100;
%Y_ref = reshape(ground_truth_map, 1, [])';
%[hyp, ~, ~] = ...
%    minimize(hyp, 'gp', N_cg, inf_func, mean_func, cov_func, lik_func, X_ref, Y_ref);


%% Measurement and Inference %%
% Generate prior map. 

%% Compute covariance (Method 1 - no inference).
% sn2=exp(2*hyp.lik);
% K = feval(cov_func{:},hyp.cov,X_ref);
%KplusR = K+ sn2*eye(length(K));
%KplusR_inv = eye(size(K))/KplusR ;
%kss = feval(cov_func{:}, hyp.cov, Z, 'diag');
%Ks = feval(cov_func{:}, hyp.cov, X_ref, Z);
%grid_map.P = diag(kss)- Ks'*KplusR_inv*Ks;

%% Compute covariance (Method 2 - with inference).
Y = reshape(grid_map_prior.m,[],1);
% ymu, ys: mean and covariance for output
% fmu, fs: mean and covariance for latent variables
% post: struct representation of the (approximate) posterior
[ymu, ys, fmu, fs, ~ , post] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y, Z);
ymu = reshape(ymu, predict_dim_y, predict_dim_x);

alpha = post.alpha;
L = post.L; 
sW = post.sW; 
kss = real(feval(cov_func{:}, hyp.cov, Z, 'diag'));
Ks = feval(cov_func{:}, hyp.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
%Lchol = 0;
%if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
%   V  = L'\(sW.*Ks);
%   grid_map_prior.P = diag(kss) - V'*V;                       % predictive variances
%  else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  grid_map_prior.P = diag(kss) + Ks'*LKs;                    % predictive variances
%end

% Extract variance map (diagonal elements).
Y_sigma = sqrt(diag(grid_map_prior.P)'); 
P_prior = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);

% Take a measurement in the centre and fuse it.
pos_env = [0, 0, 2];
grid_map_post = take_measurement_at_point(pos_env, grid_map_prior, ground_truth_map, ...
    map_parameters, planning_parameters);

Y_sigma = sqrt(diag(grid_map_post.P)'); 
P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);


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
    c1 = colorbar;
    P_climit = get(c1, 'Limits');
    colorbar off
    title('Prior variance')
    set(gca,'Ydir','Normal');
    
    subplot(1,2,2)
    contourf(P_post)
    title('Posterior variance')
    set(gca,'Ydir','Normal');
    c2 = colorbar;
    caxis(P_climit);
    set(gcf, 'Position', [752, 615, 1001, 405])
    
    %figure, surf(grid_map_post.m);
    %figure, surf(P_post);
   
end