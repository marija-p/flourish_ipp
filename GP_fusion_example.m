clear all; close all; clc;


%% Parameters %%
num_weeds = 15;

% Camera fields of view (FoV)
planning_parameters.classifier_fov_angle_x = 60;
planning_parameters.classifier_fov_angle_y = 60;
% Sensor models: polynomial coefficients
planning_parameters.weed_coeffs = [-0.000256704980842912, -0.00273180076628354, 0.912988505747127];
planning_parameters.nonweed_coeffs = [0.000233716475095785, -0.00134865900383140, 0.130114942528736];

% Map resolution [m/cell]
map_parameters.resolution = 0.5;
% Map dimensions (from ground truth)
map_parameters.env_dim_x = 10;
map_parameters.env_dim_y = 10;
% Grid dimensions
dim_x = map_parameters.env_dim_x/map_parameters.resolution;
dim_y = map_parameters.env_dim_y/map_parameters.resolution;
predict_dim_x = dim_x*1;
predict_dim_y = dim_y*1;

matlab_parameters.visualize = 1;

% GP regression parameters
cov_func = {'covMaterniso', 3};
lik_func = @likGauss;
inf_func = @infExact;
mean_func = @meanConst;
% Hyperparameters
hyp.mean = 0.5;
hyp.cov = [-1,-0.76]; %hyperpar with low correlation %[0.7337;-1.0489];
%hyp.cov = [0.1;-0.1];
%hyp.cov = log(hyp.cov);
hyp.lik = -0.7;%-1.5970; %roughly covers from 0 to 1 in 2sigma bounds
% Number of conjugate gradient steps
N_cg = -100;


%% Data %%

% Generate ground truth grid.
ground_truth_map = create_poisson_map(num_weeds, dim_x, dim_y);
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,dim_x), ...
    linspace(1,dim_y,dim_y));
X_ref = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate prediction grid.
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,predict_dim_x), ...
    linspace(1,dim_y,predict_dim_y));
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate occupancy grid.
grid_map.m = 0.5*ones(size(ground_truth_map));
%grid_map = prob_to_logodds(grid_map); %TODO remove this
entropy = get_map_entropy(grid_map.m);
disp(['Map entropy before measurements = ', num2str(entropy)])


%% Training %%
% Optimise hyperparameters.
% *** TODO (M): Not sure how to do this at the moment. ***
%Y_ref = reshape(ground_truth_map, 1, [])';
%[hyp, ~, ~] = ...
%    minimize(hyp, 'gp', N_cg, inf_func, mean_func, cov_func, lik_func, X_ref, Y_ref);


%% Measurement and Inference %%
% Generate prior map. 

%% Without performing inference we could just calculate the covariance.

% sn2=exp(2*hyp.lik);
% K = feval(cov_func{:},hyp.cov,X_ref);
%KplusR = K+ sn2*eye(length(K));
%KplusR_inv = eye(size(K))/KplusR ;
%kss = feval(cov_func{:}, hyp.cov, Z, 'diag');
%Ks = feval(cov_func{:}, hyp.cov, X_ref, Z);
%grid_map.P = diag(kss)- Ks'*KplusR_inv*Ks;

%% Performing inference.
 Y = reshape(grid_map.m,[],1);
 [ymu, ys, fmu, fs2, ~ ,post] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
     X_ref, Y, Z);
 ymu = reshape(ymu, predict_dim_y, predict_dim_x);
 ys = reshape(ys, predict_dim_y, predict_dim_x);

% we want post to calculate the covariance using Cholesky in the same way as the GPML
%Get Covariance
alpha = post.alpha; 
L = post.L; 
sW = post.sW; 
kss = feval(cov_func{:}, hyp.cov, Z, 'diag');
Ks = feval(cov_func{:}, hyp.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
   V  = L'\(sW.*Ks);
   grid_map.P = diag(kss) - V'*V;                       % predictive variances
  else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  grid_map.P = diag(kss) + Ks'*LKs;                    % predictive variances
end

%% For ploting.
Ysigma = sqrt(diag(grid_map.P)'); 
grid_map_2sigma_prior = reshape(2*Ysigma,predict_dim_y,predict_dim_x);

% Take a measurement in the centre.
pos_env = [0, 0, 4];
meas_map = take_measurement_at_point(pos_env, grid_map.m, ground_truth_map, ...
    map_parameters, planning_parameters);

% Take a measurement in the centre and fuse them.
pos_env = [0, 0, 4];
grid_map = take_measurement_at_point(pos_env, grid_map.m, ground_truth_map, ...
    map_parameters, planning_parameters, grid_map.P);
Ymean = reshape(grid_map.m,1,[])'; 
Ysigma = sqrt(diag(grid_map.P)'); 
grid_map_2sigma = reshape(2*Ysigma,predict_dim_y,predict_dim_x);


%% Plotting %%
if (matlab_parameters.visualize)
   
   subplot(1,4,1)
   imagesc([1.5, 10.5], [1.5, 10.5], ground_truth_map)
   caxis([0, 1])
   title('Ground truth weed map')
   set(gca,'Ydir','Normal');
    
   subplot(1,4,2)
   imagesc([1.5, 10.5], [1.5, 10.5], meas_map)
   caxis([0, 1])
   title('Current measurements')
   set(gca,'Ydir','Normal');
   
   subplot(1,4,3)
   imagesc([1.5, 10.5], [1.5, 10.5], ymu)
   caxis([0, 1])
   title('Prior map')
   set(gca,'Ydir','Normal');

   subplot(1,4,4)
   imagesc([1.5, 10.5], [1.5, 10.5], grid_map.m)
   caxis([0, 1])
   title('Updated map')
   set(gca,'Ydir','Normal');
   
   c = colorbar;
   set(gcf, 'Position', [58, 328, 1863, 485]);


    figure() 
    subplot(1,2,1)
    imagesc([1.5, 10.5], [1.5, 10.5],grid_map_2sigma_prior)
    title('Updated Map Variance')
    set(gca,'Ydir','Normal');
    c = colorbar;
    caxis([0, 1])
    
    subplot(1,2,2)
    imagesc([1.5, 10.5], [1.5, 10.5],grid_map_2sigma)
    title('Updated Map Variance')
    set(gca,'Ydir','Normal');
    c = colorbar;
    caxis([0, 1])

end