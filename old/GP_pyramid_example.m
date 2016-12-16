clear all; close all;

%% Parameters %%

VISUALIZE = 1;
NUM_WEEDS = 15;
PYRAMID = 2;
DimX = 20;
DimY = 20;

% Set up GP parameters.
%covfunc = 'covSEiso';
covfunc = {'covMaterniso',3};
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
% Prediction it will be at z=1, z=2 and z=3
[mesh_x,mesh_y,mesh_z] = meshgrid(linspace(1,DimX,100), ...
    linspace(1,DimY,100),[1:4]);
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1), reshape(mesh_z, numel(mesh_z), 1)];

% Create random weed map at a lower resolution.
grid_map(:,:,1) = create_poisson_map(NUM_WEEDS, DimX, DimY);
[weed_coordinates_y, weed_coordinates_x] = find(grid_map == 1);

%Create meshgrid with z In training data first layer is at z=1 and second
%layer is at z=3
[mesh_x,mesh_y,mesh_z] = meshgrid(linspace(1,DimX,DimX), ...
    linspace(1,DimY,DimY),[1,3]);

X = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1), reshape(mesh_z, numel(mesh_z), 1)];


% Create layers for multiple resolution
for i=2:PYRAMID
    tmp_map=impyramid(grid_map(:,:,i-1),'expand');
    ind = find(tmp_map > 0);
    tmp_map(ind)=1;
    grid_map(:,:,i)=tmp_map(1:2:end,1:2:end);
end

Y = reshape(grid_map,[],1);


 
%% Inference %%

% Optimise hyperparameters.
[hyp, ~, ~] = ...
    minimize(hyp, 'gp', Ncg, inffunc, meanfunc, covfunc, likfunc, X, Y); 

[ymu, ys2 , fmu, fs2] = gp(hyp, inffunc, meanfunc, covfunc, likfunc, ...
    X, Y, Z);
ymu = reshape(ymu, 100, 100,4);
ys = sqrt(reshape(ys2, 100, 100,4));

%% Plotting %%
if (VISUALIZE)
    
   % Figure 1 %
   figure
   subplot(1,2,1)
   imagesc(grid_map(:,:,1))
   title('Ground truth weed map')
   set(gca,'Ydir','Normal');
   
   subplot(1,2,2)
   imagesc(ymu(:,:,1))
   title('Interpolated weed map')
   colorbar
   set(gca,'Ydir','Normal');
   
   c = colorbar;
   caxis([0, 1])
   set(gcf, 'Position', [-1567, 331, 1247, 507]);


   % Figure 2 %
   figure
   subplot(1,2,1)
   imagesc(grid_map(:,:,2))
   title('Ground truth weed map')
   set(gca,'Ydir','Normal');
    
   subplot(1,2,2)
   imagesc(ymu(:,:,3))
   title('Interpolated weed map')
   colorbar
   set(gca,'Ydir','Normal');
   
   c = colorbar;
   caxis([0, 1])
   set(gcf, 'Position', [-1567, 331, 1247, 507]);

   for i=[2,4]
       figure
       imagesc(ymu(:,:,i))
       title('Interpolated weed map at not observed z')
       colorbar
       set(gca,'Ydir','Normal');


       figure
       imagesc(2*ys(:,:,i))
       title('Interpolated weed map at not observed z')
       colorbar
       set(gca,'Ydir','Normal');
   end
end

%%Covariances from training
K = feval(covfunc{:},hyp.cov,X);
KplusR = K+ exp(2*hyp.lik)*eye(length(K));
KplusR_inv = eye(size(K))/KplusR ;

%Covariances from testing
for i=[1:4]
    z_ind = [(i-1)*(DimX*DimY)+1:i*(DimX*DimY)]';
    kss = feval(covfunc{:},hyp.cov, Z(z_ind),'diag');
    Kst = feval(covfunc{:},hyp.cov, Z(z_ind), X) ;
    P{i} = diag(kss) - Kst*KplusR_inv*Kst';

    trace(P{i})
    if (VISUALIZE)
        figure, imagesc(P{i})
    end
end
