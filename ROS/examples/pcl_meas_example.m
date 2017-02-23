do_plot = 1;

dim_x_env = 2;
dim_y_env = 2;

transforms.T_MAP_CAM = eye(4);

[matlab_parameters, planning_parameters, ...
    optimization_parameters, map_parameters] = load_params_ros(dim_x_env, dim_y_env);

pcl_sub = rossubscriber('/pointcloud');
odom_sub = rossubscriber('/odometry');

%figure;

while (true)
  
    % Receive PCL message.
    pcl = receive(pcl_sub);
    pcl = pointCloud(readXYZ(pcl),'Color',uint8(255*readRGB(pcl)));
    pcl.Location = transforms.T_MAP_CAM * pcl.Location;
    
    % Receive odometry message.
    odom = receive(odom_sub);
    
    % Calculate submap.
    submap_edge_size = get_submap_edge_size(odom(3), map_parameters, planning_parameters);
    submap_coordinates = get_submap_coordinates(odom, submap_edge_size, map_parameters);
    submap = ...
        pcl_to_measurement(pcl, submap_edge_size, ...
        submap_coordinates, map_parameters, planning_parameters);
    
    if (do_plot)
        subplot(1,2,1)
        grid_map = zeros(map_parameters.dim_y, map_parameters.dim_x);
        grid_map(submap_coordinates.yd:submap_coordinates.yu, ...
            submap_coordinates.xl:submap_coordinates.xr) = submap;
        imagesc(grid_map)
        colorbar
        caxis([0 1])
        set(gca,'YDir','normal')
        
        subplot(1,2,2)
        pcshow(pcl)
        axis([-1 1 -1 1])
        view(2)
        drawnow;
    end
    
    %  grid_sim = pcl_to_grid(pcl, map_parameters);
  % xyz = readXYZ(pcl);
  % rgb = readRGB(pcl);
  % scatter(xyz(:,1), xyz(:,2), 5, rgb);
 %  axis([-1 1 -1 1])

 %  scatter3(pcl)
 %  axis([-2 2 -2 2])
 %  view(2)
   
 %  drawnow
 %  disp(pcl)

end