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
hyp.cov = [0, 0.1];
hyp.lik = -1.0;

% Indices of training data (can be anything within matrix range)
training_ind_x = [1:3, 8:18];
training_ind_y = [1:3, 8:18];

%% Data %%

% Generate (continuous) ground truth map.
ground_truth_map = create_continuous_map(dim_x, dim_y, cluster_radius);
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,dim_x), linspace(1,dim_y,dim_y));
X_ref = [reshape(mesh_x(training_ind_x,training_ind_x), ...
    numel(mesh_x(training_ind_x,training_ind_x)), 1), ...
    reshape(mesh_y(training_ind_y,training_ind_y), ...
    numel(mesh_y(training_ind_y,training_ind_y)), 1)];

% Generate prediction map.
[mesh_x,mesh_y] = meshgrid(linspace(1,predict_dim_x,predict_dim_x), ...
    linspace(1,predict_dim_y,predict_dim_y));
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate grid map.
grid_map_prior.m = ground_truth_map;


%% Inference %%
Y = reshape(grid_map_prior.m(training_ind_y,training_ind_x),[],1);

% ymu, ys: mean and covariance for output
% fmu, fs: mean and covariance for latent variables
% post: struct representation of the (approximate) posterior
[ymu, ys, fmu, fs, ~ , post] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y, Z);
ymu = reshape(ymu, predict_dim_y, predict_dim_x);
ys = reshape(ys, predict_dim_y, predict_dim_x);

% Calculate covariance.
alpha = post.alpha;
L = post.L; 
sW = post.sW; 
kss = real(feval(cov_func{:}, hyp.cov, Z, 'diag'));
Ks = feval(cov_func{:}, hyp.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
Lchol = 0;
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
   V = L'\(sW.*Ks);
   grid_map_prior.P = diag(kss) - V'*V;                       % predictive variances
  else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  grid_map_prior.P = diag(kss) + Ks'*LKs;                    % predictive variances
end

%% Plotting %%
if (matlab_parameters.visualize)
    
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
    imagesc(grid_map_prior.P)
    title('Covariance - prior (grid\_map\_prior)')
    colorbar;
    set(gcf, 'Position', [58, 328, 1863, 485]);
   
end