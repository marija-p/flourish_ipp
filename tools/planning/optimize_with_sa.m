function [path_optimized] = optimize_with_sa(path, grid_map, map_parameters, ...
    planning_parameters)
% Optimizes a polynomial path (defined by control points)
% using MATLAB's simulated annealing algorithm
% https://ch.mathworks.com/help/gads/simulannealbnd.html
% ---
% M Popovic 2018
%

dim_x_env = map_parameters.dim_x*map_parameters.resolution;
dim_y_env = map_parameters.dim_y*map_parameters.resolution;

% Set bounds.
LBounds = [-dim_x_env/2;-dim_y_env/2;planning_parameters.min_height];
UBounds = [dim_x_env/2;dim_y_env/2;planning_parameters.max_height];
LBounds = repmat(LBounds, size(path,1)-1, 1);
UBounds = repmat(UBounds, size(path,1)-1, 1);

% Set optimization options.
options = saoptimset('MaxIter', 500);
      
% Remove starting point (as this is fixed).
path_initial = reshape(path(2:end,:)', [], 1);
% Set objective function parameters.
f = @(path_initial)optimize_points(path_initial, path(1,:), grid_map, ...
     map_parameters, planning_parameters);
path_optimized = simulannealbnd(f, path_initial, LBounds, UBounds, options);

path_optimized = reshape(path_optimized, 3, [])';
path_optimized = [path(1,:); path_optimized];

end