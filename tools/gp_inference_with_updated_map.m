function [xf, Pf] = gp_inference_with_updated_map(y, P, GP, Xtrain, Xtest)
%% Data
% Instead of generating lets just pass it into the function
% [mesh_x,mesh_y] = meshgrid(linspace(1,r,r), linspace(1,s,s));
% X_train = [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];

%% Gaussian Process

Kss = feval(GP.cov_func{:}, GP.hyp.cov, Xtest) ;
Ks = feval(GP.cov_func{:}, GP.hyp.cov, Xtest, Xtrain);

% Inference

[L,p]  = chol(P);  % note: S = Sc'*Sc

if ~p
    
    Li = inv(L);  % note: inv(S) = Sci*Sci'
    Wc = Ks * Li;
    W  = Wc * Li';

    xf = W*(y-GP.mean_func); % update

    P = Kss - Wc*Wc';
    
else
    
    Pi=inv(P);
    
    x = Ks*Pi*(y-GP.mean_func); % update 
        % x = x + PHt/S*v; % update
    P = Kss -  Ks*Pi*Ks';
    % P = P - PHt/S*PHt';
    
end

