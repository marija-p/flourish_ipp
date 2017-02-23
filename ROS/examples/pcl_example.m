%global pcl

dim_x_env = 4;
dim_y_env = 4;

[matlab_parameters, planning_parameters, ...
    optimization_parameters, map_parameters] = load_params_ros(dim_x_env, dim_y_env);

pcl_sub = rossubscriber('/pointcloud');

%figure;

while (true)
  
    % Receive message.
    pcl = receive(pcl_sub);
    pcl = pointCloud(readXYZ(pcl),'Color',uint8(255*readRGB(pcl)));
    % Convert to grid.
    grid = pcl_to_grid(pcl, map_parameters);
    
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