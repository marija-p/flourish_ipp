function [xf,P] = correlated_gridmap_fusion(xa,P,za,Ra, ind)
%Maximum aposteriori estimate given a prior map x and P and a set of
%measurments z R and the indices of the measurement in the grid_map
%
% Teresa Vidal Calleja 12/2016
%
%

[m,n] = size(xa);
x = reshape(xa,[],1);

[r,s] = size(za);
z = reshape(za,[],1);
R = diag(reshape(Ra,[],1));

H = zeros(r*s,m*n);
j=1;
for i=1:length(ind)
    H(j,ind(i)) = 1;
    j=j+1;
end
[x,P] = KF_update_cholesky(x,P,z-H*x,R,H);

xf = reshape(x,m,n);

end
