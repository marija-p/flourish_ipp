function grid_mean = pcl_to_grid(pcl, map_parameters)

do_plot = 1;

% Get map dimensions [cells].
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;

% Read data.
xyz = pcl.Location;
xyz = env_to_grid_coordinates(xyz, map_parameters);
rgb = pcl.Color;

% Remove invalid values.
xyz = xyz(~isnan(xyz(:,1)),:);
rgb = rgb(~isnan(rgb(:,1)),:);

% Remove values outside environment range.
remove_idx = (xyz(:,1) < 1 | xyz(:,2) < 1 | xyz(:,3) < 0 | ...
    xyz(:,1) > dim_y | xyz(:,2) > dim_x);
xyz(remove_idx, :) = [];

grid_sum = zeros(dim_y, dim_x);
grid_numel = zeros(dim_y, dim_x);

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

