function [submap] = get_downsampled_submap(height, submap) 

[resy,resx] = size(submap);

factor = get_resolution_from_height(height);

tmp_map=imresize(submap,1/factor);
submap = imresize(tmp_map,[resy,resx],'nearest');