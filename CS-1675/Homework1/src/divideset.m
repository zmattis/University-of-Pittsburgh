% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 1
%  * January 23, 2019
%  *
%  * This Matlab file is a function that splits
%  * a dataset utilizing the traiing probability
%  * into two non-overlapping datasets: training & testing.
%  */

function [ training_set, testing_set ] = divideset( dataset, p_train )

training_count = round(p_train * length(dataset));

indices = randperm(length(dataset), training_count);

t = zeros([length(dataset) 1]);

t(indices) = 1;

training_set = dataset(t(:) == 1,:);
testing_set = dataset(t(:) == 0,:);

end