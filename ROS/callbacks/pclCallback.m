function pclCallback(~, msg, map_parameters)

global pcl

pcl = msg;
grid_sim = pcl_to_grid(pcl, map_parameters);

end