clear all; close all;

%% Parameters %%
NUM_WEEDS = 15;

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
DimX = map_parameters.env_dim_x/map_parameters.resolution;
DimY = map_parameters.env_dim_y/map_parameters.resolution;

matlab_parameters.visualize = 1;

% GP regression parameters
%covfunc = 'covSEiso';
covfunc = {'covMaterniso', 5};
likfunc = @likGauss;
inffunc = @infExact;
meanfunc = @meanConst;
% Hyperparameters
hyp.mean = 0.5;
hyp.cov = [0.2; 0.3];
hyp.cov = log(hyp.cov);
hyp.lik = 1;
% Number of conjugate gradient steps
Ncg = -100;

%% Data %%
% Generate prediction grid
[mesh_x,mesh_y] = meshgrid(linspace(1,21,210), ...
    linspace(1,21,210));
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate ground truth grid
ground_truth_map = create_poisson_map(NUM_WEEDS, DimX, DimY);
[mesh_x,mesh_y] = meshgrid(linspace(1,DimX,DimX), ...
    linspace(1,DimY,DimY));
% Use it to train hyperparameters.
X_ref = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)] + 0.5;
Y_ref = reshape(ground_truth_map, 1, [])';
% Optimise hyperparameters.
[hyp, ~, ~] = ...
    minimize(hyp, 'gp', Ncg, inffunc, meanfunc, covfunc, likfunc, X_ref, Y_ref);

% Generate occupancy grid.
grid_map = 0.5*ones(size(ground_truth_map));
grid_map = prob_to_logodds(grid_map);

%% Measurement and Inference %%

% Take a measurement in the centre.
pos_env = [0, 0, 3];
[grid_map] = take_measurement_at_point(pos_env, grid_map, ground_truth_map, ...
    map_parameters, planning_parameters);
Y = reshape(logodds_to_prob(grid_map),1,[])';

% Perform inference.
[ymu, ys2 , fmu, fs2] = gp(hyp, inffunc, meanfunc, covfunc, likfunc, ...
    X_ref, Y, Z);
ymu = reshape(ymu, 210, 210);

%% Plotting %%
if (matlab_parameters.visualize)
   
   subplot(1,3,1)
   imagesc([1.5, 10.5], [1.5, 10.5], ground_truth_map)
   title('Ground truth weed map')
   set(gca,'Ydir','Normal');
    
   subplot(1,3,2)
   imagesc([1.5, 10.5], [1.5, 10.5], logodds_to_prob(grid_map))
   title('Current measurements')
   set(gca,'Ydir','Normal');
   
   subplot(1,3,3)
   imagesc([1.05, 110.5], [1.05, 110.5], ymu)
   title('Interpolated  measurements')
   set(gca,'Ydir','Normal');
   
   c = colorbar;
   caxis([0, 1])
   set(gcf, 'Position', [58, 328, 1863, 485]);

end