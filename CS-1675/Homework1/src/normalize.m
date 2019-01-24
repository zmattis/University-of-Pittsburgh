% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 1
%  * January 23, 2019
%  *
%  * This Matlab file is a function that normalizes
%  * a given attribute vector according to its
%  * data mean and standard deviation.
%  */

function [ normalized, mu, sigma ] = normalize( attribute )

    mu = mean(attribute);    
    sigma = std(attribute);
    normalized = (attribute - mu)/sigma;

end