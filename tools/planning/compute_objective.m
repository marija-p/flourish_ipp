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

% Create polynomial path through the control points.
trajectory = ...
    plan_path_waypoints(control_points, planning_parameters.max_vel, planning_parameters.max_acc);

% Sample trajectory to find locations to take measurements at.
[t, measurement_points, ~, ~] = ...
    sample_trajectory(trajectory, ...
    1/planning_parameters.measurement_frequency);

P_i = trace(grid_map.P);

% Predict measurements along the path.
% NB: - No measurement taken at first (current) point!
for i = 2:size(measurement_points,1)
    grid_map = predict_map_update(measurement_points(i,:), grid_map, ...
        map_parameters, planning_parameters);
end

P_f = trace(grid_map.P);
gain = P_i - P_f;
cost = t(end);

% Formulate objective.
obj = -gain*exp(-planning_parameters.lambda*cost);

%disp(['Measurements = ', num2str(i)])
%disp(['Gain = ', num2str(gain)])
%disp(['Cost = ', num2str(cost)])
%disp(['Objective = ', num2str(obj)])

end