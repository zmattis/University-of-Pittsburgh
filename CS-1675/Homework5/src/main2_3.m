% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 5
%  * February 21, 2019
%  *
%  * This Matlab file predicts the class labels
%  * and computes the misclassification error
%  * of the Naive Bayers classifier on both
%  * the pima_*.txt training and test datasets.
%  */

% exp_0_muhat 
% exp_0_muci

% norm_0_mu
% norm_0_sigma
% norm_0_muci
% norm_0_sci

load pima_train.txt;
load pima_test.txt;

pima = pima_train;

main2_2;

exp_mu = [exp_0_muhat exp_1_muhat];
norm_mu = [norm_0_mu norm_1_mu];
norm_sigma = [norm_0_sigma norm_1_sigma];

% -- Calculate train error
classes = zeros(size(pima_train,1),1);

for i=1:size(pima_train,1)
    [g0, g1, p0, p1] = predict_NB(pima_train(i,:), exp_mu, norm_mu, norm_sigma);
    if g0 > g1
        classes(i) = 0;
    else
        classes(i) = 1;
    end
end

Y = pima_train(:,9);
errs = classes - Y;
FN = sum(errs < 0);
FP = sum(errs > 0);
%sum(errs == 0)
TP = sum(Y(errs == 0) == 1);
TN = sum(Y(errs == 0) == 0);

SENS_train = TP/(TP + FN);
SPEC_trian = TN/(TN + FP);
mis_train = (FP + FN)/(TP + TN);
confmat_train = [TP FP; FN TN];

% -- Calculate Test error
classes = zeros(size(pima_test,1),1);

for i=1:size(pima_test,1)
    [g0, g1, p0, p1] = predict_NB(pima_test(i,:), exp_mu, norm_mu, norm_sigma);
    if g0 > g1
        classes(i) = 0;
    else
        classes(i) = 1;
    end
end

Y = pima_test(:,9);
errs = classes - Y;
FN = sum(errs < 0);
FP = sum(errs > 0);
%sum(errs == 0)
TP = sum(Y(errs == 0) == 1);
TN = sum(Y(errs == 0) == 0);

SENS_test = TP/(TP + FN);
SPEC_test = TN/(TN + FP);
mis_test = (FP + FN)/(TP + TN);
confmat_test = [TP FP; FN TN];