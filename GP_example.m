clear all; close all;

%% Parameters %%

VISUALIZE = 1;
NUM_WEEDS = 15;

% Set up GP parameters.
covfunc = 'covSEiso';
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

% Generate prediction grid.
[mesh_x,mesh_y] = meshgrid(linspace(1,11,110), ...
    linspace(1,11,110));
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Create random weed map at a lower resolution.
grid_map = create_poisson_map(NUM_WEEDS, 10, 10);
[weed_coordinates_y, weed_coordinates_x] = find(grid_map == 1);

[mesh_x,mesh_y] = meshgrid(linspace(1,10,10), ...
    linspace(1,10,10));
X = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)] + 0.5;
Y = reshape(grid_map,1,[])';

%% Inference %%

% Optimise hyperparameters.
[hyp, ~, ~] = ...
    minimize(hyp, 'gp', Ncg, inffunc, meanfunc, covfunc, likfunc, X, Y); 

[ymu, ys2 , fmu, fs2] = gp(hyp, inffunc, meanfunc, covfunc, likfunc, ...
    X, Y, Z);
ymu = reshape(ymu, 110, 110);


%% Plotting %%
if (VISUALIZE)
   
   subplot(1,2,1)
   imagesc([1.5, 10.5], [1.5, 10.5], grid_map)
   title('Ground truth weed map')
   set(gca,'Ydir','Normal');
    
   subplot(1,2,2)
   imagesc([1.05, 110.5], [1.05, 110.5], ymu)
   title('Interpolated weed map')
   colorbar
   set(gca,'Ydir','Normal');
   
   c = colorbar;
   caxis([0, 1])
   set(gcf, 'Position', [-1567, 331, 1247, 507]);

end

