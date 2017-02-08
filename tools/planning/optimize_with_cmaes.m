function [path_optimized] = optimize_with_cmaes(path, grid_map, map_parameters, ...
    planning_parameters, optimization_parameters)
% Optimizes a polynomial path (defined by control points) using Covariance
% Matrix Adaptation Evolutionary Strategy (CMA-ES).
% ---
% M Popovic 2017
%

dim_x_env = map_parameters.dim_x*map_parameters.resolution;
dim_y_env = map_parameters.dim_y*map_parameters.resolution;

% Set optimization parameters.
opt = cmaes;
opt.DispFinal = 'off';
opt.LogModulo = 0;
opt.TolFun = 1e-9;
opt.IncPopSize = 1; %% Check this
opt.PopSize = 30;
opt.SaveVariables = 'off';
opt.MaxIter = optimization_parameters.max_iters;
opt.Seed = randi(2^10);

% Set bounds and covariances.
LBounds = [-dim_x_env/2;-dim_y_env/2;planning_parameters.min_height];
UBounds = [dim_x_env/2;dim_y_env/2;planning_parameters.max_height];
opt.LBounds = repmat(LBounds, size(path,1)-1, 1);
opt.UBounds = repmat(UBounds, size(path,1)-1, 1); 
cov = [optimization_parameters.cov_x; optimization_parameters.cov_y; optimization_parameters.cov_z];
cov = repmat(cov, size(path,1)-1, 1);

% Remove starting point (as this is fixed).
path_initial = reshape(path(2:end,:)', [], 1);
path_optimized = cmaes('optimize_points', path_initial, cov, opt, path(1,:), ...
    grid_map, map_parameters, planning_parameters);
path_optimized = reshape(path_optimized, 3, [])';
path_optimized = [path(1,:); path_optimized];

end

