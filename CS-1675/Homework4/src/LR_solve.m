% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 4
%  * February 13, 2019
%  *
%  * This Matlab file is a function that computes
%  * a vector with the minimal mean square fit
%  * from an input matrix and vector.
%  */
function [ w ] = LR_solve( X, y )

    w = X\y;

end