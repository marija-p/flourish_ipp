function H = construct_H(x, z, submap_coordinates, altitude)
% Constructs the measurement model (H) for the KF, accounting for
% varying resolution with altitude.
% ---
% Marija Popovic 2017

% Get dimensions.
[m,n] = size(x);
[r,s] = size(z);
H = zeros(r*s,m*n);
res_factor = get_resolution_from_height(altitude);

% Altitude resolution diffusion effect:
% Find the starting indices of the "diffused" cells.
if (res_factor ~= 1)
    [submap_ind_x, submap_ind_y] = ...
        meshgrid(submap_coordinates.xl:res_factor:submap_coordinates.xr,...
        submap_coordinates.yd:res_factor:submap_coordinates.yu);
else
    [submap_ind_x, submap_ind_y] = ...
        meshgrid(submap_coordinates.xl:submap_coordinates.xr,...
        submap_coordinates.yd:submap_coordinates.yu);
end

submap_ind = sub2ind(size(x),reshape(submap_ind_y,[],1), reshape(submap_ind_x,[],1));

% Loop through all the measurements.
% Identify the states corresponding to a measurement, and index these in
% the model.
for i=1:length(submap_ind)
    
    [loc_y, loc_x] = ind2sub([m n], submap_ind(i));
    [locs_x, locs_y] = meshgrid(loc_x:min(loc_x+res_factor-1,submap_coordinates.xr),...
        loc_y:min(loc_y+res_factor-1,submap_coordinates.yu));
    indices = sub2ind([m n], reshape(locs_y,[],1),reshape(locs_x,[],1));
    
    if (loc_x == submap_coordinates.xr || loc_y == submap_coordinates.yu)
        H(i, indices) = 1/res_factor;
    else
        H(i, indices) = 1/(res_factor^2);
    end
    
end

end