function [metrics] = rigtree_fun(matlab_parameters, planning_parameters, ...
    optimization_parameters, map_parameters, ground_truth_map)
% Main program for IROS2017 RIG-tree algorithm benchmark.
%
% M Popovic 2017
%

% Initialize variables.

% Get map dimensions [cells]
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;
% Set prediction map dimensions [cells]
predict_dim_x = dim_x*1;
predict_dim_y = dim_y*1;

% Gaussian Process
cov_func = {'covMaterniso', 3};
lik_func = @likGauss;
inf_func = @infExact;
mean_func = @meanConst;
% Hyperparameters
hyp.mean = 0.5;
hyp.cov =  [1.3 0.3];
hyp.lik =  0.35;

% First measurement location
point_init = [7.5, 7.5, 8.66];
 
%% Data %%
% Generate (continuous) ground truth map.
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,dim_x), linspace(1,dim_y,dim_y));
X_ref = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate prediction map.
[mesh_x,mesh_y] = meshgrid(linspace(1,predict_dim_x,predict_dim_x), ...
    linspace(1,predict_dim_y,predict_dim_y));
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

% Generate grid map.
grid_map.m = 0.5*ones(size(ground_truth_map));


%% Initial Measurement and Inference %%
% Generate prior map.
Y = reshape(grid_map.m,[],1);

% ymu, ys: mean and covariance for output
% fmu, fs: mean and covariance for latent variables
% post: struct representation of the (approximate) posterior
[ymu, ys, fmu, fs, ~ , post] = gp(hyp, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y, Z);
ymu = reshape(ymu, predict_dim_y, predict_dim_x);

alpha = post.alpha;
L = post.L; 
sW = post.sW;
Kss = real(feval(cov_func{:}, hyp.cov, Z));
Ks = feval(cov_func{:}, hyp.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
  V = L'\(sW.*Ks);
  grid_map.P = Kss - V'*V;                       % predictive variances
 else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  grid_map.P = Kss + Ks'*LKs;                    % predictive variances
end

Y_sigma = sqrt(diag(grid_map.P)');
P_post = reshape(2*Y_sigma,predict_dim_y,predict_dim_x);
P_trace_init = trace(grid_map.P);

time_elapsed = 0;
metrics = initialize_metrics();




% Initialize first vertex in tree.
rigtree_planner = RIGTree();
start_vertex = Vertex();
start_vertex.location = pos_env;
start_vertex.cost = 0;

% Initialize best vertex.
I_best = Inf;

% Initialize metrics.
logger = create_logger();

%% Planning %%
% Timer for total simulation time
t_total = tic;
% Total time spent on planning
planning_time = 0;

%% Planning-Execution Loop %%

while (true)
    
    t_plan = tic;
    
    start_vertex.map = grid_map;
    start.vertex.information = get_map_entropy(grid_map);
    rigtree_planner = rigtree_planner.resetTree();
    rigtree_planner.rigtree.vertices = start_vertex;
    rigtree_planner.rigtree.numvertices = 1;
    rigtree_planner.setStart(start_vertex);
    found_best_node = 0;
    
    if (matlab_parameters.visualize)
        delete(h_tree);
    end
    
    for j = 1:500
        
        % Sample configuration space of UAV and find nearest node.
        x_samp = rigtree_planner.sampleLocation();
        [q_nearest, ~] = rigtree_planner.findNearestVertex(x_samp);
        x_feasible = rigtree_planner.stepToLocation(q_nearest.location, x_samp);
        
        % Find near points to be extended.
        neighbors_idx = rigtree_planner.findNeighborIndices(x_feasible);
        
        if (isempty(neighbors_idx))
            continue;
        end
        
        for i = 1:size(neighbors_idx)
            
            q_near = rigtree_planner.rigtree.vertices(neighbors_idx(i));
            
            % Extend towards new point.
            q_new = Vertex();
            q_new.location = rigtree_planner.stepToLocation(q_near.location, x_feasible);
            
            % Calculate new information and cost.
            [I_new, grid_map_new] = q_new.evaluateInformation(q_near, ...
                map_parameters, planning_parameters);
            C_new = q_near.cost + q_new.evaluateCost(q_near.location, ...
                planning_parameters);
            q_new.information = I_new;
            q_new.cost = C_new;
            q_new.map = grid_map_new;
            
            % Check if target node should be pruned.
            if (rigtree_planner.pruneVertex(q_new))
                disp(['Pruned node: x = ', num2str(q_new.location(1)), ...
                    ', y = ', num2str(q_new.location(2)), ', z = ', num2str(q_new.location(3))])
                continue;
            else
                % Add edges and node to tree.
                rigtree_planner.rigtree.vertices = ...
                    [rigtree_planner.rigtree.vertices; q_new];
                rigtree_planner.rigtree.numvertices = ...
                    rigtree_planner.rigtree.numvertices + 1;
                rigtree_planner.rigtree.edges = [rigtree_planner.rigtree.edges; ...
                    rigtree_planner.addEdge(neighbors_idx(i), q_new)];
            end
            
            % Update best solution.
            if (I_new < I_best)
                q_best_idx = rigtree_planner.rigtree.numvertices;
                I_best = I_new;
                found_best_node = 1;
            end
            
            % Add to closed list if budget exceeded.
            if (C_new > planning_parameters.time_budget)
                rigtree_planner.rigtree.vertices_closed = ...
                    [rigtree_planner.rigtree.vertices_closed; q_new];
            end
            
            if (matlab_parameters.visualize)
                point1 = env_to_grid_coordinates(q_near.location, map_parameters);
                point2 = env_to_grid_coordinates(q_new.location, map_parameters);
                h_tree = [h_tree, plot3([point1(1), point2(1)], ...
                    [point1(2), point2(2)], ...
                    [point1(3), point2(3)], 'Color', 'g', 'LineWidth', 1)];
                drawnow;
            end
            
        end
        
    end
    
    if (~found_best_node)
        break;
    end
    
    %% Execution and Logging %%
    
    % Find and draw the most informative path.
    start_vertex = rigtree_planner.rigtree.vertices(q_best_idx);
    current_path = rigtree_planner.tracePath(q_best_idx);
    
    % Update metrics for the planned path.
    % We need to go through all points in the path and re-compute the
    % metrics using real measurements.
    %disp(toc(t_plan));
    planning_time = planning_time + toc(t_plan);

    for l = 1:size(current_path,2)
        
        % Budget has been spent.
        if (current_path(l).cost > planning_parameters.time_budget)
            current_path = current_path(1:l-1);
            budget_spent = 1;
            break;
        end
        
        % Take a real measurement at a viewpoint using the reference bitmap.
        [grid_map, ~] = take_measurement_at_viewpoint(current_path(l).location, ...
            current_path(l).location, grid_map, ground_truth, map_parameters, planning_parameters);

        logger.classification_rate = [logger.classification_rate; ...
            get_map_class_rate(grid_map, map_parameters)];
        logger.entropy = [logger.entropy; get_map_entropy(grid_map)];
        logger.f1_score = [logger.f1_score; get_map_f1_score(grid_map, ...
            ground_truth, map_parameters)];
        logger.f2_score = [logger.f2_score; get_map_f2_score(grid_map, ...
            ground_truth, map_parameters)];
        logger.f05_score = [logger.f05_score; get_map_f05_score(grid_map, ...
            ground_truth, map_parameters)];
        logger.accuracy = [logger.accuracy; get_map_accuracy(grid_map, ...
            ground_truth, map_parameters)];
        logger.time_zeroopt = [logger.time_zeroopt; current_path(l).cost];
        logger.time_nonzeroopt = [logger.time_nonzeroopt; ...
            current_path(l).cost + planning_time];
        
    end
    
    logger.path = [logger.path, current_path];
    
    if (matlab_parameters.visualize)
        h_path = rigtree_planner.plotPath(current_path, map_parameters);
        % Also display the final map.
        h_grid = imagesc(logodds_to_prob(grid_map));
    end
    
    if (budget_spent)
        break;
    end
    
end

logger.sim_time = toc(t_total);

if (matlab_parameters.visualize)
    delete(h_tree)
end

disp(['Planning took ', num2str(logger.sim_time), 's.']);
disp(['Path cost: ', num2str(rigtree_planner.rigtree.vertices(q_best_idx).cost)]);
disp(['Map entropy: ', num2str(rigtree_planner.rigtree.vertices(q_best_idx).information)]);

end