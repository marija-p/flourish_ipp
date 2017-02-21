function [xf,Pf] = fuse_measurements(xa,P,za,Ra,submap_coordinates,altitude)
% Obtains maximum aposteriori estimate using Bayesian Fusion.
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
% Marija Popovic 2017
%

% Construct matrices for KF update.
x = reshape(xa,[],1);
z = reshape(za,[],1);
R = diag(reshape(Ra,[],1));
H = construct_H(xa, za, submap_coordinates, altitude);

[x,Pf] = KF_update_cholesky(x,P,z-H*x,R,H);

xf = reshape(x, size(xa,1), size(xa,2));

end