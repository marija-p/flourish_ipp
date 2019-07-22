% Start ROS comms.
pose_pub = rospublisher('/firefly/command/pose', ...
        rostype.geometry_msgs_PoseStamped);
pose_msg = rosmessage(pose_pub);

odom_sub = rossubscriber('/firefly/ground_truth/odometry');

% Transforms
%transforms.T_MAP_W = [1 0 0 2; 0 1 0 2; 0 0 1 0; 0 0 0 1];
transforms.T_MAP_W = eye(4);
%transforms.T_VSB_CAM = eye(4);
transforms.T_VSB_CAM = [1 0 0 0; 0 1 0 0; 0 0 1 -0.3; 0 0 0 1];

% Distance before a waypoint is considered reached.
achievement_dist = 0.2;

% Some path
point_init = [0 0 0.5];
path = [0 0 2; 1 2 3; 0 0 4; -2 1 0.3];

% Send the command.
target_T_MAP_CAM = trvec2tform(point_init);
target_T_MAP_CAM(3,3) = -1;
target_T_W_VSB = ...
    get_inv_transform(transforms.T_MAP_W)* ...
    target_T_MAP_CAM*get_inv_transform(transforms.T_VSB_CAM);
target_point = tform2trvec(target_T_W_VSB);
pose_msg.Pose.Position.X = target_point(1);
pose_msg.Pose.Position.Y = target_point(2);
pose_msg.Pose.Position.Z = target_point(3);
send(pose_pub, pose_msg)

% Go to target point.
reached_point = false;
while (~reached_point)
    odom = receive(odom_sub);
    % Ignore orientation for now.
    x_odom_W_VSB = [odom.Pose.Pose.Position.X, ...
        odom.Pose.Pose.Position.Y, odom.Pose.Pose.Position.Z];
    T_W_VSB = trvec2tform(x_odom_W_VSB);
    T_MAP_CAM = transforms.T_MAP_W * T_W_VSB * transforms.T_VSB_CAM;
    point_odom_MAP_CAM = tform2trvec(T_MAP_CAM);
    if (pdist2(point_init, point_odom_MAP_CAM) < achievement_dist)
        reached_point = true;
    end
end

while (true)
    
    %% Plan Execution %%
    % Create polynomial trajectory through the control points.
    trajectory = plan_path_waypoints(path, 1.5, 2);

    % Sample trajectory to find locations to take measurements at.
    [times_meas, points_meas, ~, ~] = sample_trajectory(trajectory, 1/0.15);
    
    disp(points_meas)
    
    % Take measurements along path, updating the grid map.
    for i = 1:size(points_meas,1)
        
        % Send the command.
        target_T_MAP_CAM = trvec2tform(points_meas(i,:));
        target_T_MAP_CAM(3,3) = -1;
        target_T_W_VSB = ...
            get_inv_transform(transforms.T_MAP_W)* ...
            target_T_MAP_CAM*get_inv_transform(transforms.T_VSB_CAM);
        target_point = tform2trvec(target_T_W_VSB);
        disp(target_point)
        pose_msg.Pose.Position.X = target_point(1);
        pose_msg.Pose.Position.Y = target_point(2);
        pose_msg.Pose.Position.Z = target_point(3);
        send(pose_pub, pose_msg)
        
        % Go to target point.
        reached_point = false;
        while (~reached_point)
            odom = receive(odom_sub);
            % Ignore orientation for now.
            x_odom_W_VSB = [odom.Pose.Pose.Position.X, ...
                odom.Pose.Pose.Position.Y, odom.Pose.Pose.Position.Z];
            T_W_VSB = trvec2tform(x_odom_W_VSB);
            T_MAP_CAM = transforms.T_MAP_W * T_W_VSB * transforms.T_VSB_CAM;
            point_odom_MAP_CAM = tform2trvec(T_MAP_CAM);
            if (pdist2(points_meas(i,:), point_odom_MAP_CAM) < achievement_dist)
                reached_point = true;
            end
        end
        
    end
    
end