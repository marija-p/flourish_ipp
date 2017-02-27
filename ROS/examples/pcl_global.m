% Receive PCL messages and visualize them in the global frame.

dim_x_env = 2;
dim_y_env = 2;

transforms.T_MAP_W = eye(4);
% transforms.T_VSB_CAM = ...
%     [-0.0010, -1.0000, -0.0009, -0.0140;
%    -1.0000,    0.0010,    0.0005,   -0.0000;
%    -0.0005,    0.0009,   -1.0000,   -0.0559;
%          0,         0,         0,    1.0000];

transforms.T_VSB_CAM = ...
    [0, -1.0000, 0, -0.0140;
    -1.0000,    0,    0,   -0.0000;
     0,    0,   -1.0000,   -0.0559;
          0,         0,         0,    1.0000];

[matlab_parameters, planning_parameters, ...
    optimization_parameters, map_parameters] = ...
    load_params_ros(dim_x_env, dim_y_env);

pcl_sub = rossubscriber('/pointcloud');
odom_sub = rossubscriber('/flourish/vrpn_client/estimated_odometry');

grid_sum = zeros(20);
grid_numels = zeros(20);

while (true)
    
    % Receive messages.
    pcl = receive(pcl_sub);
    odom = receive(odom_sub);
    delay = (pcl.Header.Stamp.Nsec - odom.Header.Stamp.Nsec)*10^-9;
    if (abs(delay) > 0.07)
        continue;
    end
    disp(['Delay = ', num2str(delay), 's'])
    
    % Get transform: map -> camera.
    x_odom_W_VSB = [odom.Pose.Pose.Position.X, ...
        odom.Pose.Pose.Position.Y, odom.Pose.Pose.Position.Z];
    quat_odom_W_VSB = [odom.Pose.Pose.Orientation.W, ...
        odom.Pose.Pose.Orientation.X, odom.Pose.Pose.Orientation.Y, ...
        odom.Pose.Pose.Orientation.Z];
    T_W_VSB = quat2tform(quat_odom_W_VSB);
    T_W_VSB(1:3, 4) = x_odom_W_VSB';
    T_MAP_CAM = transforms.T_MAP_W * T_W_VSB * transforms.T_VSB_CAM;
    
    % Get transform: map -> points.
    pcl = pointCloud(readXYZ(pcl),'Color',uint8(255*readRGB(pcl)));
    pcl = pctransform(pcl, affine3d(T_MAP_CAM'));
    
    % Discritize this and display the grid.
    grid = pcl_to_grid(pcl, map_parameters);
    grid(isnan(grid)) = 0;
    grid_sum = grid_sum + grid;
    grid_numels(find(grid)) = grid_numels(find(grid)) + 1;
    
end

grid_ave = grid_sum./grid_numels;
figure; imagesc(grid_ave);
colorbar
set(gca, 'YDir', 'Normal')