% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 5
%  * February 21, 2019
%  *
%  * This Matlab file is used to calculate the
%  * confusion matrix, misclassification error, 
%  * sensitivity, and specificity for both the 
%  * training and testing "pima" set.
%  */

load pima_train.txt;
load pima_test.txt;

% ---- Train ---- %

x_train = pima_train(:,1:8);
y_train = pima_train(:,9);

[w, b] = svml(x_train, y_train, 1)

results = zeros(size(x_train,1),1);
for i=1:length(results)
    results(i) = apply_svml(x_train(i,:), w, b);
end

errs = results - y_train;
FN = sum(errs < 0);
FP = sum(errs > 0);
%sum(errs == 0)
TP = sum(y_train(errs == 0) == 1);
TN = sum(y_train(errs == 0) == 0);

SENS_train = TP/(TP + FN)
SPEC_trian = TN/(TN + FP)
MIS_train = (FP + FN)/(TP + TN + FP + FN)
CONFMAT_train = [TP FP; FN TN]

% ---- Test ---- %

x_test = pima_test(:,1:8);
y_test = pima_test(:,9);

results = zeros(size(x_test,1),1);
for i=1:length(results)
    results(i) = apply_svml(x_test(i,:), w, b);
end

errs = results - y_test;
FN = sum(errs < 0);
FP = sum(errs > 0);
%sum(errs == 0)
TP = sum(y_test(errs == 0) == 1);
TN = sum(y_test(errs == 0) == 0);

SENS_test = TP/(TP + FN)
SPEC_test = TN/(TN + FP)
MIS_test = (FP + FN)/(TP + TN + FP + FN)
CONFMAT_test = [TP FP; FN TN]
