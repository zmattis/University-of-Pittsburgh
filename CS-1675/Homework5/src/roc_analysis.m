% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 5
%  * February 21, 2019
%  *
%  * This Matlab file conducts ROC
%  * analysis on two different models,
%  * logistic regression and naive bayes.
%  */

load pima_train_norm.txt;
load pima_test_norm.txt;
load pima_train.txt;
load pima_test.txt;

disp('Part a')
X = pima_train_norm(:,1:8);
W = Log_regression(X, pima_train(:,9), 200);

X = pima_test_norm(:,1:8);
col_ones = ones(size(X, 1), 1);
X = horzcat(col_ones, X);

scores = zeros(size(X, 1),1);
for row=1:1:size(X,1)
    scores(row) = 1/(1+exp(-(W'*X(row,:)')));
end

labels = (pima_test(:,9) == 1);

[X,Y,T,AUC] = perfcurve(labels, scores, 1);
plot(X,Y);
%plot([0 1], [0 1]);
%plot([AUC AUC], [0 1]);

AUC

xlabel('False Positive Rate')
ylabel('True Positive Rate')
title('ROC - Logistic Regression')


disp('Part b')

pima = pima_train;

main2_2;

exp_mu = [exp_0_muhat exp_1_muhat];
norm_mu = [norm_0_mu norm_1_mu];
norm_sigma = [norm_0_sigma norm_1_sigma];

classes = zeros(size(pima_test,1),1);

for i=1:size(pima_test,1)
    [g0, g1, p0, p1] = predict_NB(pima_test(i,:), exp_mu, norm_mu, norm_sigma);
    classes(i) = g1;
end

figure();
[X,Y,T,AUC] = perfcurve(labels, classes, 1);
plot(X,Y);
hold on;
%plot([0 1], [0 1]);
%plot([AUC AUC], [0 1]);
xlabel('False Positive Rate')
ylabel('True Positive Rate')
title('ROC - Naive Bayes')