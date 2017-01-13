function [] = plot_path(path)
% Creates a new figure and to visualize list of travelled waypoints.

colors = linspace(1, 10, size(path,1));

hold on
cline(path(:,1), path(:,2), path(:,3), colors);
scatter3(path(:,1), path(:,2), path(:,3), 100, colors, 'filled');

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis([-15 15 -15 15 0 30])
grid minor
colormap jet
view(3)

end

