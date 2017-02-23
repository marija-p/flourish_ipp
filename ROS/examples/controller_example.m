% Start ROS comms.
pose_pub = rospublisher('/firefly/command/pose', ...
        rostype.geometry_msgs_PoseStamped);
pose_msg = rosmessage(pose_pub);

odom_sub = rossubscriber('/firefly/ground_truth/odometry');

% Distance before a waypoint is considered reached.
achievement_dist = 0.2;

% Some path
point_init = [0 0 0.5];
path = [0 0 2; 1 2 3; 0 0 4; -2 1 0.3];

% Send the command.
pose_msg.Pose.Position.X = point_init(1);
pose_msg.Pose.Position.Y = point_init(2);
pose_msg.Pose.Position.Z = point_init(3);
send(pose_pub, pose_msg)

% Go to target point.
reached_point = false;
while (~reached_point)
    odometry = receive(odom_sub);
    point_odometry = [odometry.Pose.Pose.Position.X, ...
        odometry.Pose.Pose.Position.Y, odometry.Pose.Pose.Position.Z];
    if (pdist2(point_init, point_odometry) < achievement_dist)
        reached_point = true;
    end
end

while (true)
    
    %% Plan Execution %%
    % Create polynomial trajectory through the control points.
    trajectory = plan_path_waypoints(path, 2, 1);

    % Sample trajectory to find locations to take measurements at.
    [times_meas, points_meas, ~, ~] = sample_trajectory(trajectory, 5);
    
    disp(points_meas)
    
    % Take measurements along path, updating the grid map.
    for i = 1:size(points_meas,1)
        
        % Send the command.
        pose_msg.Pose.Position.X = points_meas(i,1);
        pose_msg.Pose.Position.Y = points_meas(i,2);
        pose_msg.Pose.Position.Z = points_meas(i,3);
        send(pose_pub, pose_msg)
        
        % Go to target point.
        reached_point = false;
        while (~reached_point)
            odometry = receive(odom_sub);
            point_odometry = [odometry.Pose.Pose.Position.X, ...
                    odometry.Pose.Pose.Position.Y, odometry.Pose.Pose.Position.Z];
            if (pdist2(points_meas(i,:), point_odometry) < achievement_dist)
                reached_point = true;
            end
        end
        
    end
    
end