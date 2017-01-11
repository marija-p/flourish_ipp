function [x,P]= KF_update_cholesky_Cov_only(P,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P], the innovation v, the 
% observe uncertainty R, and the (linearised) observation model H. The result is calculated 
% using Cholesky factorisation, which is more numerically stable than a naive implementation.
%
% Adapted from code by Jose Guivant.
%
% Tim Bailey 2003. Updated T Vidal-Calleja 2017.

PHt = P*H';
S = H*PHt + R;

S = (S+S')*0.5; % ensure S is symmetric 

[Sc,p]  = chol(S);  % note: S = Sc'*Sc

if ~p
    Sci = inv(Sc);  % note: inv(S) = Sci*Sci'
    Wc = PHt * Sci;
    W  = Wc * Sci';

    P = P - Wc*Wc';
else
    Si=inv(S);
    P = P - PHt*Si*H*P;
    % P = P - PHt/S*PHt';
end