function [metrics, grid_map] = GP_rig(matlab_parameters, planning_parameters, ...
    map_parameters, subtree_iters, ground_truth_map)
% Main program for IROS2017 RIG-tree algorithm benchmark.
%
% M Popovic 2017
%

% Initialize variables.

%matlab_parameters.visualize = 1;
planning_parameters = rmfield(planning_parameters, 'control_points');   % For plotting

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

budget_spent = 0;
time_elapsed = 0;
metrics = initialize_metrics();

% Initialize first vertex in tree.
rigtree_planner = RIGTree();
q_start = Vertex();
q_start.location = point_init;

if (matlab_parameters.visualize)
    fig1 = figure;
    axis([-15 15 -15 15 0 28])
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    h_tree = [];
    grid on
    grid minor
    hold on 
end


%% Planning-Execution Loop %%
while (true)
    
 %   keyboard
    
    % Initialize a tree.
    rigtree_planner = rigtree_planner.resetTree();
    rigtree_planner.rigtree.vertices = q_start;
    rigtree_planner.rigtree.numvertices = 1;
    rigtree_planner.setStart(q_start);
    found_best_node = 0;
    obj_best = Inf;
    
    %% Sub-Tree Planning %%
    for j = 1:subtree_iters
        
        % Sample configuration space and find nearest node.
        x_samp = rigtree_planner.sampleLocation();
        [q_nearest, ~] = rigtree_planner.findNearestVertex(x_samp);
        x_feasible = rigtree_planner.stepToLocation(q_nearest.location, x_samp);
        
        % Find near points to be extended.
        neighbors_idx = rigtree_planner.findNeighborIndices(x_feasible);
        
        if (isempty(neighbors_idx))
            continue;
            keyboard
        end
        
        for i = 1:size(neighbors_idx)
            
            q_near = rigtree_planner.rigtree.vertices(neighbors_idx(i));
            
            % Extend towards new point.
            q_new = Vertex();
            q_new.location = rigtree_planner.stepToLocation(q_near.location, x_feasible);
            
            % Calculate new information and cost.
            q_new = q_new.evaluateObjective(neighbors_idx(i), rigtree_planner, grid_map, ...
                map_parameters, planning_parameters);
            
            % Check if target vertex should be pruned.
            %if (rigtree_planner.pruneVertex(q_new))
            %    disp(['Pruned node: x = ', num2str(q_new.location(1)), ...
            %        ', y = ', num2str(q_new.location(2)), ', z = ', num2str(q_new.location(3))])
            %    continue;
            %else
                % Add edges and node to tree.
                rigtree_planner.rigtree.vertices = ...
                    [rigtree_planner.rigtree.vertices; q_new];
                rigtree_planner.rigtree.numvertices = ...
                    rigtree_planner.rigtree.numvertices + 1;
                rigtree_planner.rigtree.edges = [rigtree_planner.rigtree.edges; ...
                    rigtree_planner.addEdge(neighbors_idx(i), q_new)];
            %end
            
            % Update best solution.
            if (q_new.objective < obj_best)
                q_best_idx = rigtree_planner.rigtree.numvertices;
                obj_best = q_new.objective;
                found_best_node = 1;
            end
            
            % Add to closed list if budget exceeded.
            if (q_new.cost > planning_parameters.time_budget)
                rigtree_planner.rigtree.vertices_closed = ...
                    [rigtree_planner.rigtree.vertices_closed; q_new];
            end
            
            if (matlab_parameters.visualize)
                point1 = q_near.location;
                point2 = q_new.location;
                h_tree = [h_tree, plot3([point1(1), point2(1)], ...
                    [point1(2), point2(2)], ...
                    [point1(3), point2(3)], 'Color', [0.7, 0.7, 0.7], 'LineWidth', 1)];
                drawnow;
            end
            
        end
        
    end
    
    if (~found_best_node)
        break;
    end
    
    %% Sub-tree Plan Execution %%
    
    % Find and draw the most informative path.
    q_start = rigtree_planner.rigtree.vertices(q_best_idx);
    vertices_current = rigtree_planner.tracePath(q_best_idx);
    path_current = rigtree_planner.getVertexLocations(vertices_current);

    % Create polynomial trajectory through the control points.
    trajectory = ...
        plan_path_waypoints(path_current, planning_parameters.max_vel, planning_parameters.max_acc);
    
    % Sample trajectory to find locations to take measurements at.
    [times_meas, points_meas, ~, ~] = ...
        sample_trajectory(trajectory, 1/planning_parameters.measurement_frequency);

    % Update metrics for the planned path.
    % Take measurements along path, updating the grid map.
    for i = 1:size(points_meas,1)

        % Budget has been spent.
        if ((time_elapsed + times_meas(i)) > planning_parameters.time_budget)
            points_meas = points_meas(1:i-1,:);
            times_meas = times_meas(1:i-1);
            budget_spent = 1;
            break;
        end

        grid_map = take_measurement_at_point(points_meas(i,:), grid_map, ...
            ground_truth_map, map_parameters, planning_parameters);
        metrics.P_traces = [metrics.P_traces; trace(grid_map.P)];
        metrics.rmses = [metrics.rmses; compute_rmse(grid_map.m, ground_truth_map)];
        metrics.wrmses = [metrics.wrmses; compute_wrmse(grid_map.m, ground_truth_map)];
        metrics.mlls = [metrics.mlls; compute_mll(grid_map, ground_truth_map)];
        metrics.wmlls = [metrics.wmlls; compute_wmll(grid_map, ground_truth_map)];
        
    end

    metrics.points_meas = [metrics.points_meas; points_meas];
    metrics.times = [metrics.times; time_elapsed + times_meas'];
    metrics.path_travelled = [metrics.path_travelled; path_current];
    
    time_elapsed = time_elapsed + get_trajectory_total_time(trajectory);  

    if (budget_spent)
        break;
    end
    
end

end