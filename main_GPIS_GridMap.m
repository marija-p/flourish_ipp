%% Data
Worldx = 10;
Worldy = 10;
Worldz = 3;

GridMap=zeros(Worldx,Worldy,Worldz);
xy=ceil(10*rand(15,2));

map = robotics.BinaryOccupancyGrid(Worldx,Worldy,1);
setOccupancy(map,xy,1);
figure; show(map);

GridMap(xy(:,1),xy(:,2),1)=1;
GridMap(:,:,2)=GridMap(:,:,1)-1;
GridMap(:,:,3)=GridMap(:,:,1)-2;

locX=[1:Worldx];
locY=[1:Worldy];
locZ=[1:Worldz];

[gridX,gridY,gridZ]=meshgrid(locX,locY,locZ);

xtrain=reshape(gridX,1,[])';
ytrain=reshape(gridY,1,[])';
ztrain=reshape(gridZ,1,[])';

SDFvaluetrain = reshape(GridMap,1,[])';
%%  GP training (use LR data) 
xtr = [xtrain ytrain ztrain];
target  = SDFvaluetrain ; % dsZL ; 
offset = mean(target);
ytr = target - offset;

%== training
sn = 100; 
mean1 = {@meanZero}; hyp0.mean = [];
cov1  = {'covMaterniso',3}; sigf = 4; ell = 8;  hyp0.cov = log([ell;sigf]);
Ncg = 250;                                   % number of conjugate gradient step

lik = 'likGauss';% setup the likelihood
inf = 'infExact';

if strcmp(lik,'likT')
    nu = 4;
    hyp0.lik  = log([nu-1;sqrt((nu-2)\nu)*sn]);
elseif strcmp(lik,'likErf');
      hyp0.lik  = [];
else
    hyp0.lik  = log(sn);
end
if Ncg==0 
  hyp1 = hyp0;
else
  [hyp1,fvals,iter] = minimize(hyp0,'gp', -Ncg, inf, mean1, cov1, lik, xtr, ytr); % opt hypers   
  disp('new hyp=');    disp(exp(hyp1.cov));
end


%% GPs Prediction on training
[ymu, ys2 , fmu, fs2] = gp(hyp1, inf, mean1, cov1, lik, xtr, ytr, xtr);
absDiff = abs(fmu+offset - SDFvaluetrain(:));
rmse = sqrt( sum(absDiff.^2)/length(absDiff) )
if rmse>40
    error('Oops, it seems GP does not converge well, Please re-run the code to give GP another try!');
end

%% Covariance Matrix
K = feval(cov1{:},hyp1.cov,xtr);
P= K+ exp(2*hyp1.lik)*eye(length(K));

figure, imshow(P(1:Worldx*Workly,1:Worldx*Workly))