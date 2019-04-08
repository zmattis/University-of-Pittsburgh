% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 10
%  * April 8, 2019
%  *
%  * This Matlab file is a function that runs
%  * ROC score algorithm used to rank different
%  * dimensions of inputs for a given vector.
%  */

function [ score ] = AUROC_score( X, y )

    [X,y,T,AUC] = perfcurve(y, X, 1);
    score = AUC;

end