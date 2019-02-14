% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 4
%  * February 13, 2019
%  *
%  * This Matlab file computes the testing and
%  * training data set utilizing the online
%  * gradient descent procedure.
%  */

load housing_test.txt;
load housing_train.txt;

g = init_progress_graph();
g.pause=0.1;

train_trials = 100000000;

test  = housing_test;
train = housing_train;

mu = mean(train);
sigma = std(train);


train_n = (train-mu)./sigma;
test_n  = (test -mu)./sigma;

w = zeros(length(train_n(1,:))-1,1);

y_test = LR_predict(test_n(:,1:13), w);
y_train = LR_predict(train_n(:,1:13),w);
mse_train = sum((y_train - train_n(:,14)).^2)/length(train_n(:,14));
mse_test  = sum((y_test - test_n(:,14)).^2)/length(test_n(:,14));
g = add_to_progress_graph(g, 0, mse_train, mse_test);

online_gradient_descent();

y_test = LR_predict(test_n(:,1:13), w);
y_train = LR_predict(train_n(:,1:13),w);
mse_train = sum((y_train - train_n(:,14)).^2)/length(train_n(:,14))
mse_test  = sum((y_test - test_n(:,14)).^2)/length(test_n(:,14))

w