function path = search_lattice(point_init, lattice, grid_map, map_parameters, ...
    planning_parameters)
% Performs a greedy grid search over a list of candidates to identify
% most promising points to visit based on an informative objective.
% Starting point is fixed (no measurement taken here)
% ---
% Inputs:
% point_init: starting location
% lattice: list of candidates to evaluate
% grid_map: current grid map (mean + covariance)
% ---
% Output:
% path: grid search result
% ---
% M Popovic 2017
%

P_trace_prev = trace(grid_map.P);
point_prev = point_init;
path = point_init;

% First measurement?
%grid_map = predict_map_update(point_init, grid_map, ...
%    map_parameters, planning_parameters);
        
while (planning_parameters.control_points > size(path, 1))
    
    % Initialise best solution so far.
    obj_min = Inf;
    point_best = -Inf;
    
    for i = 1:size(lattice, 1)
        
        point_eval = lattice(i, :);
        grid_map_eval = predict_map_update(point_eval, grid_map, ...
            map_parameters, planning_parameters);
        P_trace = trace(grid_map_eval.P);
        
        gain = P_trace_prev - P_trace;
        
        if (strcmp(planning_parameters.obj, 'rate'))
            cost = max(pdist([point_prev; point_eval])/planning_parameters.max_vel, ...
                1/planning_parameters.measurement_frequency);
            %path_eval = [path; point_eval];
            %trajectory = plan_path_waypoints(path_eval, ...
            %    planning_parameters.max_vel, planning_parameters.max_acc);
            %cost = max(get_trajectory_total_time(trajectory), ...
            %    1/planning_parameters.measurement_frequency);
            obj = -gain/cost;
        elseif (strcmp(planning_parameters.obj, 'exponential'))
            cost = pdist([point_prev; point_eval])/planning_parameters.max_vel;
            obj = -gain*exp(-planning_parameters.lambda*cost);
        end
        
        %disp(['Point ', num2str(point_eval)]);
        %disp(['Gain: ', num2str(gain)])
        %disp(['Cost: ', num2str(cost)])
        %disp(num2str(obj));
        
        % Update best solution.
        if (obj < obj_min)
            obj_min = obj;
            point_best = point_eval;
        end
        
    end
    
    % Update the map with measurement at best point.
    grid_map = predict_map_update(point_best, grid_map, ...
        map_parameters, planning_parameters);
    disp(['Point ', num2str(size(path,1)+1), ' at: ', num2str(point_best)]);
    disp(['Trace of P: ', num2str(trace(grid_map.P))]);
    disp(['Objective: ', num2str(obj_min)]);
    path = [path; point_best];
    
    P_trace_prev = trace(grid_map.P);
    point_prev = point_best;
    
end

end