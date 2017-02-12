function obj = compute_objective(control_points, grid_map, map_parameters,...
    planning_parameters)
% Calculates the expected informative objective for a polynomial path.
% ---
% Inputs:
% control_points: list of waypoints defining the polynomial
% grid_map: current map
% ---
% Output:
% obj: informative objective value (to be minimized)
% ---
% M Popovic 2017
%

% Create polynomial path through the control points.
trajectory = ...
    plan_path_waypoints(control_points, planning_parameters.max_vel, planning_parameters.max_acc);

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
    obj = Inf;
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
gain = P_i - P_f;
%cost = get_trajectory_total_time(trajectory);
%obj = -gain*exp(-planning_parameters.lambda*cost);
cost = max(get_trajectory_total_time(trajectory), 1/planning_parameters.measurement_frequency);
obj = -gain/cost;

%disp(['Measurements = ', num2str(i)])
%disp(['Gain = ', num2str(gain)])
%disp(['Cost = ', num2str(cost)])
%disp(['Objective = ', num2str(obj)])

end