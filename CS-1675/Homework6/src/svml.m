%%% converts 0,1 class labels to -1,1
%%  and calls Lagrangian SVM code by O. L. Mangasarian and D.R. Musicant
%% 
%% input: Xs,Ys, and costs for crossing the margin
%% output: weights + bias term   wx+b=0 is the separating hyperplane

function [w, b] = svml(X,Y_in,cost)

%%% convert 0 1 class labels to -1 +1 labels
Y=2*Y_in - ones(size(Y_in,1),1);
D=diag(Y);;
A=X;
nu=cost;
%% itmax
itmax=10000;
%% tolerance
tol=0.005;
%%% call Lagrangian SVM by O. L. Mangasarian and D.R. Musicant
[it, opt, w, b] = svml_itsol(A,D,nu,itmax,tol);

return;