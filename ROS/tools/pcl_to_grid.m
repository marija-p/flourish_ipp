function grid_mean = pcl_to_grid(pcl, map_parameters)

do_plot = 0;

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

% Also get HSV colormap.
hsv = rgb2hsv(double(rgb)./255);

% Remove values outside environment range.
remove_idx = (xyz(:,1) < 1 | xyz(:,2) < 1 | ...
    xyz(:,2) > dim_y | xyz(:,1) > dim_x);
xyz(remove_idx, :) = [];

grid_sum = NaN(dim_y, dim_x);
grid_numel = NaN(dim_y, dim_x);

for i = 1:size(xyz,1)
    
    if isnan(grid_sum(xyz(i,2),xyz(i,1)))
        grid_sum(xyz(i,2),xyz(i,1)) = hsv(i,2);
        grid_numel(xyz(i,2),xyz(i,1)) = 1;
    else
        grid_sum(xyz(i,2),xyz(i,1)) = grid_sum(xyz(i,2),xyz(i,1)) + hsv(i,2);
        grid_numel(xyz(i,2),xyz(i,1)) = grid_numel(xyz(i,2),xyz(i,1)) + 1;
    end
    
end

% Set threshold for min. number of points falling within a cell.
ind = find(grid_numel < 15);
grid_mean(ind) = NaN;

grid_mean = grid_sum./grid_numel;
grid_mean = grid_mean.*6;
%grid_mean = grid_mean./255;

if (do_plot)
    subplot(1,2,1)
    hold on
    sc(grid_mean, [0 1], 'parula', 'k');
    colorbar
    caxis([0 1])
    xlabel('x (m)')
    ylabel('y (m)')
    set(gca,'YDir','normal')

    subplot(1,2,2)
    pcshow(pcl)
    axis([-1 1 -1 1 -1 6])
    xlabel('x (m)')
    ylabel('y (m)')
    view(2)
    drawnow;
end

end

