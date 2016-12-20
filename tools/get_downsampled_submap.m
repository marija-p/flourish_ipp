function [submap] = get_downsampled_submap(altitude, submap) 
% Obtains submap measurements at a coarser resolution depending on UAV altitude.

[res_y, res_x] = size(submap);

factor = get_resolution_from_height(altitude);

tmp_map = imresize(submap,1/factor);
submap = imresize(tmp_map,[res_y,res_x],'nearest');