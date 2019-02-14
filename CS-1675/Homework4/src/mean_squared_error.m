% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 4
%  * February 13, 2019
%  *
%  * This Matlab file computes the mean squared
%  * error of both training and testing data sets.
%  */

load housing_test.txt;
load housing_train.txt;

test = housing_test;
train = housing_train;

w_train = LR_solve(train(:,1:13), train(:,14));
disp(w_train)

y_train = LR_predict(train(:,1:13), w_train);
y_test = LR_predict(test(:,1:13), w_train);

mse_train = sum((y_train - train(:,14)).^2)/length(train(:,14))
mse_test = sum((y_test - test(:,14)).^2)/length(test(:,14))