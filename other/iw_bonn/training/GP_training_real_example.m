clear all; close all; clc;

%% Parameters %%

% Environment
load('image3m_00002.mat');
[dim_x dim_y]=size(hue);

matlab_parameters.visualize = 1;

% Gaussian Process
cov_func = {'covMaterniso', 3};
lik_func = @likGauss;
inf_func = @infExact;
mean_func = @meanConst;
% Hyperparameters
hyp.mean = 0.5;
hyp.cov = [1,1];   % With low correlation
hyp.lik = 5;         % Roughly covers from 0 to 1 in 2*sigma bounds


%% Data %%

% Generate (continuous) ground truth map.
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,dim_x), linspace(1,dim_y,dim_y));

X_ref = [];
Y_ref = [];
for i = 1:1
    for j = 1:1
        ground_truth_map = hue;
        %ground_truth_map = grid_ave/0.77;0.
        X_ref = [X_ref; (i)*(dim_x*1.5)+reshape(mesh_x, numel(mesh_x), 1), (j)*(dim_x*1.5)+reshape(mesh_y, numel(mesh_y), 1)];
        Y_ref = [Y_ref; reshape(ground_truth_map,[],1)];
    end
end
%% Training %%
% Optimise hyperparameters.
% Number of conjugate gradient steps
N_cg = -100;
[hyp, ~, ~] = ...
    minimize(hyp, 'gp', N_cg, inf_func, mean_func, cov_func, lik_func, X_ref, Y_ref);
disp(hyp)

%%
%Testing
%%
%new hyp
hyp1 = hyp;
hyp1.cov = [0.7328 -0.75];
hyp1.lik = -0.35;

Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];
[ymu, ys, fmu, fs, ~ , post] = gp(hyp1, inf_func, mean_func, cov_func, lik_func,X_ref, Y_ref,Z);

alpha = post.alpha;
L = post.L; 
sW = post.sW;
sW(1)
Kss = real(feval(cov_func{:}, hyp1.cov, Z));
Ks = feval(cov_func{:}, hyp1.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
  V = L'\(sW.*Ks);
  grid_map.P = Kss - V'*V;                       % predictive variances
 else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  grid_map.P = Kss + Ks'*LKs;                    % predictive variances
end

sn2=exp(2*hyp1.lik);
K = feval(cov_func{:},hyp1.cov,X_ref);
KplusR = K+ sn2*eye(length(K));
KplusR_inv = eye(size(K))/KplusR ;
Kss = feval(cov_func{:},hyp1.cov, Z) ;
Kst = feval(cov_func{:},hyp1.cov, Z, X_ref) ;
grid_map.PP = Kss - Kst*KplusR_inv*Kst';

% Extract variance map (diagonal elements).
Y_sigma = sqrt(diag(grid_map.P)'); 
P_prior = reshape(2*Y_sigma,dim_y,dim_x);

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
   
%     
    % Variances
    %figure;
    subplot(1,3,3)
    contourf(P_prior)
    c1 = colorbar;
    P_climit = get(c1, 'Limits');
    colorbar off
    title('Prior variance')
    set(gca,'Ydir','Normal');
    
    
    %figure, surf(grid_map_post.m);
    %figure, surf(P_prior);
   

figure, imagesc(grid_map.P);
%figure, imagesc(grid_map.PP);
end
