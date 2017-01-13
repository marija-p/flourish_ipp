function [xf,Pf] = fuse_measurements(xa,P,za,Ra,ind,altitude)
% Obtain maximum aposteriori estimate using Bayesian Fusion.
%
% Inputs:
% x: prior mean
% P: prior covariance
% z: measurements
% R: measurement variances
% ind: measurement indices in grid map
% altitude: UAV altitude [m]
% ---
% Outputs:
% xf: map with fused measurements
% Pf: covariance with fused measurements
% ---
% Teresa Vidal Calleja 12/2016
%

% Construct matrices for KF update.
[m,n] = size(xa);
x = reshape(xa,[],1);

[r,s] = size(za);
z = reshape(za,[],1);
R = diag(reshape(Ra,[],1));

% Construct measurement model, accounting for variable resolution measurements.
H = zeros(r*s,m*n);
res_factor = get_resolution_from_height(altitude);

% Loop through all the measurements.
% Altitude resolution diffusion effect:
% Identify the states corresponding to a measurement, and index these in
% the model.
for i=1:length(ind)
    
 %   try
    [loc_y, loc_x] = ind2sub([m n], ind(i));
    [locs_x, locs_y] = meshgrid(loc_x:loc_x+res_factor-1,...
        loc_y:loc_y+res_factor-1);
    indices = sub2ind([m n], reshape(locs_y,[],1),reshape(locs_x,[],1));
%    catch
%        keyboard
%    end
    
    H(i, indices) = 1/(res_factor^2);
    
end

[x,Pf] = KF_update_cholesky(x,P,z-H*x,R,H);

xf = reshape(x,m,n);

end