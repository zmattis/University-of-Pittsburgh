% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 1
%  * January 23, 2019
%  *
%  * This Matlab file is a function that discretizes
%  * a given attribute vector into one of k bins.
%  */

function [ discrete ] = discretize_attribute( attribute, k )

    min_val = min(attribute);
    max_val = max(attribute);
    
    bin_div = (max_val - min_val)/k;
    
    discrete = fix((attribute-min_val)/bin_div);

end