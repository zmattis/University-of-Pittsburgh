% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 10
%  * April 8, 2019
%  *
%  * This Matlab file is a function that runs
%  * Fisher score algorithm used to rank different
%  * dimensions of inputs for a given vector.
%  *
%  * Fisher(i) =     ({mu_+} - {mu_-})^2
%  *              -------------------------
%  *              {sigma_+}^2 + {sigma_-}^2
%  */

function [ score ] = Fisher_score( X, y )

    x_0 = X(y == 0);
    x_1 = X(y == 1);

    score = (mean(x_0) - mean(x_1))^2/(std(x_0)^2 + std(x_1)^2);

end
