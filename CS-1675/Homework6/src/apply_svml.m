% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 6
%  * February 27, 2019
%  *
%  * This Matlab file is a function that calculates the
%  * class decision of a Linear SVM, using parameters:
%  *     x - vector
%  *     w - weight
%  *     b - bias
%  */

function [d] = apply_svml(x, w, b)
% input vector x, parameters weight w, bias b

if w'*x' + b >= 0
    d = 1;
else
    d = 0;
end

end