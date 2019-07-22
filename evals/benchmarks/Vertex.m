classdef Vertex
    
    properties
        
        % Coordinates in configuration space (x,y,z)
        location;
        % Informative objective at this point
        gain;
        cost;
        objective;
        
    end
    
    methods
        
        % Evaluates informative objective at a vertex.
        function self = evaluateObjective(self, q_near_idx, tree, grid_map, ...
                map_parameters, planning_parameters)
            
            % Add current vertex to the tree.
            tree.rigtree.vertices = ...
                [tree.rigtree.vertices; self];
            tree.rigtree.numvertices = ...
                tree.rigtree.numvertices + 1;
            tree.rigtree.edges = [tree.rigtree.edges; ...
                tree.addEdge(q_near_idx, self)];
            
            % Find path to current vertex in tree.
            control_vertices = tree.tracePath(tree.rigtree.numvertices);
            control_points = tree.getVertexLocations(control_vertices);
            trajectory = ...
                plan_path_waypoints(control_points, ...
                planning_parameters.max_vel, planning_parameters.max_acc);

            % Sample trajectory to find locations to take measurements at.
            [~, points_meas, ~, ~] = ...
            sample_trajectory(trajectory, ...
                1/planning_parameters.measurement_frequency);

            if (planning_parameters.use_threshold)
                above_thres_ind = find(grid_map.m >= planning_parameters.lower_threshold);
                P = reshape(diag(grid_map.P)', size(grid_map.m));
                P_i = sum(P(above_thres_ind));
            else
                P_i = trace(grid_map.P);
            end

            % Discard path if it is too long.
            if (size(points_meas,1) > 10)
                self.objective = Inf;
                return;
            end

            % Predict measurements along the path.
            for i = 1:size(points_meas,1)
                grid_map = predict_map_update(points_meas(i,:), grid_map, ...
                    map_parameters, planning_parameters);
            end

            if (planning_parameters.use_threshold)
                P = reshape(diag(grid_map.P)', size(grid_map.m));
                P_f = sum(P(above_thres_ind));
            else
                P_f = trace(grid_map.P);
            end

            % Formulate objective.
            self.gain = P_i - P_f;
            self.cost = get_trajectory_total_time(trajectory);
            self.objective = -self.gain*exp(-planning_parameters.lambda*self.cost);

        end
        
        function r = eq(a, b)
            
            if (isempty(b))
                r = false;
                return;
            end
            
            if (a.location == b.location)
                r = true;
            else
                r = false;
            end
            
        end
        
    end
    
end

