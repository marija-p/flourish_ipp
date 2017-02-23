function grid_mean = pcl_to_measurement(pcl, submap_edge_size, ...
    submap_coordinates, map_parameters, planning_parameters)

do_plot = 0;

% Read data.
xyz = pcl.Location;
xyz = env_to_grid_coordinates(xyz, map_parameters);
rgb = pcl.Color;

% Remove invalid values.
xyz = xyz(~isnan(xyz(:,1)),:);
rgb = rgb(~isnan(rgb(:,1)),:);

% Remove values outside environment range.
remove_idx = (xyz(:,1) < submap_coordinates.yd | ...
    xyz(:,2) < submap_coordinates.xl | ....
    xyz(:,3) < planning_parameters.min_height | ...
    xyz(:,1) > submap_coordinates.yu | ...
    xyz(:,2) > submap_coordinates.xr | ...
    xyz(:,3) > planning_parameters.max_height);
xyz(remove_idx, :) = [];

grid_sum = zeros(submap_edge_size.y, submap_edge_size.x);
grid_numel = zeros(submap_edge_size.y, submap_edge_size.x);

for i = 1:size(xyz,1)
    
    grid_sum(xyz(i,2),xyz(i,1)) = grid_sum(xyz(i,2),xyz(i,1)) + double(rgb(i,2));
    grid_numel(xyz(i,2),xyz(i,1)) = grid_numel(xyz(i,2),xyz(i,1)) + 1;
    
end

grid_mean = grid_sum./grid_numel;
grid_mean = grid_mean./255;

if (do_plot)
    subplot(1,2,1)
    imagesc(grid_mean);
    colorbar
    caxis([0 1])
    set(gca,'YDir','normal')

    subplot(1,2,2)
    pcshow(pcl)
    axis([-2 2 -2 2])
    view(2)
    drawnow;
end

end

