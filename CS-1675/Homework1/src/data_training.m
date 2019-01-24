% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 1
%  * January 23, 2019
%  *
%  * This Matlab file performs data training on a given
%  * input file, pima.txt.
%  */

load pima.txt;


test_count = 0;
train_count = 0;

for i=1:20
    [train, test] = divideset(pima, 0.66);
    test_count = test_count + size(test, 1);
    train_count = train_count + size(train, 1);
end


test_count = test_count / 20
train_count = train_count / 20

test_chance = test_count/size(pima,1)
train_chance = train_count/size(pima,1)