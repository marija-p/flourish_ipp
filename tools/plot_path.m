function [] = plot_path(path, planning_parameters)
% Visualizes a trajectory from a list of control points.

trajectory = ...
    plan_path_waypoints(path, planning_parameters.max_vel, ...
    planning_parameters.max_acc);
[t, p] = sample_trajectory(trajectory, 0.1);
[~, p_meas] = sample_trajectory(trajectory, 1/planning_parameters.measurement_frequency);

hold on
cline(p(:,1), p(:,2), p(:,3), t);
scatter3(path(:,1), path(:,2), path(:,3), 140, 'xk');
colors_meas = linspace(0, t(end),size(p_meas,1));
scatter3(p_meas(:,1), p_meas(:,2), p_meas(:,3), 60, colors_meas, 'filled');

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis([-15 15 -15 15 0 30])
grid minor
colormap jet
view(3)

end

