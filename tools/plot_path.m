function [] = plot_path(path, planning_parameters)
% Visualizes a trajectory from a list of control points.

t = [];
p = [];
p_meas = [];

% Loop through all polynomial trajectories that were executed,
% and stack times/measurements for plotting.
for i = 1:planning_parameters.control_points:size(path,1)
    
    trajectory = ...
        plan_path_waypoints(path(i:i+3,:), planning_parameters.max_vel, ...
        planning_parameters.max_acc);
    [t_poly, p_poly] = sample_trajectory(trajectory, 0.1);
    [~, p_meas_poly] = sample_trajectory(trajectory, ...
        1/planning_parameters.measurement_frequency);
    if (i == 1)
        t = [t; t_poly'];
    else
        t = [t; t(end) + t_poly'];
    end
    p = [p; p_poly];
    p_meas = [p_meas; p_meas_poly];
    
end

hold on
% Visualize trajectory.
cline(p(:,1), p(:,2), p(:,3), t);
% Visualize control points.
scatter3(path(:,1), path(:,2), path(:,3), 140, 'xk');
% Visualize measurements.
colors_meas = linspace(0, t(end),size(p_meas,1));
scatter3(p_meas(:,1), p_meas(:,2), p_meas(:,3), 60, colors_meas, 'filled');

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis([-20 20 -20 20 0 35])
grid minor
colormap jet
c = colorbar;
ylabel(c, 'Time (s)')
view(3)
legend('Path', 'Control pts.', 'Meas. pts.', 'Location', 'northeast')

end

