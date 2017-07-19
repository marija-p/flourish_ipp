function [submap] = get_downsampled_submap_ros(altitude, submap) 
% Obtains submap measurements at a coarser resolution depending on UAV altitude.

res_factor = get_resolution_from_height_ros(altitude);

tmp_map = imresize(submap, 1/res_factor);
submap = tmp_map;

% Upsample submap to highest resolution.
%[res_y, res_x] = size(submap);
%submap = imresize(tmp_map, [res_y,res_x], 'nearest');