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
cov_func = {'covMaterniso', 5};
lik_func = @likGauss;
inf_func = @infExact;
mean_func = @meanConst;
% Hyperparameters
hyp.mean = 0.5;
hyp.cov = [1;7];
hyp.cov = log(hyp.cov);
hyp.lik = 0.9;
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
grid_map = 0.5*ones(size(ground_truth_map));
%grid_map = prob_to_logodds(grid_map); %TODO remove this
entropy = get_map_entropy(grid_map);
disp(['Map entropy before measurements = ', num2str(entropy)])


%% Training %%
% Optimise hyperparameters.
% *** TODO (M): Not sure how to do this at the moment. ***
%Y_ref = reshape(ground_truth_map, 1, [])';
%[hyp, ~, ~] = ...
%    minimize(hyp, 'gp', N_cg, inf_func, mean_func, cov_func, lik_func, X_ref, Y_ref);


%% Measurement and Inference %%
% Generate prior map. 
% Perform inference.
%Y = reshape(logodds_to_prob(grid_map),1,[])'; %TODO remove this
Y = reshape(grid_map,1,[])';
[ymu, ys, fmu, fs2] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y, Z);
ymu = reshape(ymu, predict_dim_x, predict_dim_y);
ys = reshape(ys, predict_dim_x, predict_dim_y);

%Get Covariance



% Take a measurement in the centre.
pos_env = [0, 0, 4];
[grid_map] = take_measurement_at_point(pos_env, grid_map, ground_truth_map, ...
    map_parameters, planning_parameters);
Y = reshape(logodds_to_prob(grid_map),1,[])';

% Perform inference.
[ymu, ys, fmu, fs2] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y, Z);
ymu = reshape(ymu, predict_dim_x, predict_dim_y);
ys = reshape(ys, predict_dim_x, predict_dim_y);

% Discretize the posterior (?).
grid_map_interp = imresize(ymu, 0.1, 'nearest');
grid_map_interp = prob_to_logodds(grid_map_interp);
entropy = get_map_entropy(grid_map_interp);
disp(['Map entropy after 1 meas. = ', num2str(entropy)])

%% Plotting %%
if (matlab_parameters.visualize)
   
   subplot(1,4,1)
   imagesc([1.5, 10.5], [1.5, 10.5], ground_truth_map)
   caxis([0, 1])
   title('Ground truth weed map')
   set(gca,'Ydir','Normal');
    
   subplot(1,4,2)
   imagesc([1.5, 10.5], [1.5, 10.5], logodds_to_prob(grid_map))
   caxis([0, 1])
   title('Current measurements')
   set(gca,'Ydir','Normal');
   
   subplot(1,4,3)
   imagesc([1.05, 110.5], [1.05, 110.5], ymu)
   caxis([0, 1])
   title('Interpolated measurements')
   set(gca,'Ydir','Normal');

   subplot(1,4,4)
   imagesc([1.05, 110.5], [1.05, 110.5], logodds_to_prob(grid_map_interp))
   caxis([0, 1])
   title('Interpolated measurements')
   set(gca,'Ydir','Normal');
   
   c = colorbar;
   set(gcf, 'Position', [58, 328, 1863, 485]);

end

%% Next Measurement Site Selection %%
% Create list of possible sites at which to take measurements.
pos_candidates = [0, 0, 4; 5, 2, 4];

if (matlab_parameters.visualize)
    figure;
end

for i = 1:size(pos_candidates,1)
    
    [grid_map_eval] = take_measurement_at_point(pos_candidates(i,:), ...
        grid_map_interp, ground_truth_map, ...
        map_parameters, planning_parameters);
    Y = reshape(logodds_to_prob(grid_map_eval),1,[])';
    entropy = get_map_entropy(grid_map_eval);
    disp(['Map entropy for candidate pos. ', num2str(i), ' = ', num2str(entropy)])
    
    if (matlab_parameters.visualize)
        subplot(1,2,i)
        title(['Map entropy = ', num2str(entropy)]);
        imagesc([1.05, 110.5], [1.05, 110.5], logodds_to_prob(grid_map_eval))
    end
    
    [ymu, ys, fmu, fs2] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
        X_ref, Y, Z);
    ymu = reshape(ymu, predict_dim_x, predict_dim_y);
    ys = reshape(ys, predict_dim_x, predict_dim_y);
    
end
