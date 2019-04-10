% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 10
%  * April 9, 2019
%  *
%  * This Matlab file is a driver file used
%  * to compare the performace of the
%  * base SVM model and the decision tree
%  * model via training and testing errors.
%  */

load hw10_train.txt;
load hw10_test.txt;

tr_x = hw10_train(:,1:65);
tr_y = hw10_train(:,66);
test_x = hw10_test(:,1:65);
test_y = hw10_test(:,66);

err_test_bag = zeros(10,1);
err_train_bag = zeros(10,1);

for i=1:10
    for j=1:20
        [t_y] = Bag_classifier(tr_x,tr_y,test_x,sprintf('[@DT_base_full,%d,[]]',i));
        err_test_bag(i) = err_test_bag(i) + sum(t_y == test_y)/size(test_y,1);
        [tr_y2] = Bag_classifier(tr_x,tr_y,tr_x,sprintf('[@DT_base_full,%d,[]]',i));
        err_train_bag(i) = err_train_bag(i) + sum(tr_y2 == tr_y)/size(tr_y,1);
    end

    err_test_bag(i) = 1-err_test_bag(i)/20;
    err_train_bag(i) = 1-err_train_bag(i)/20;

end

[err_test_bag'; err_train_bag'];

figure();
plot(1:10,err_test_bag')
hold on;
plot(1:10,err_train_bag')
legend('Testing Error', 'Training Error');
title('Decision Tree');
xlabel('T');
ylabel('Error');
