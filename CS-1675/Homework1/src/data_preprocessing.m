% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 1
%  * January 23, 2019
%  *
%  * This Matlab file performs data preprocessing on a given
%  * input file, pima.txt.
%  */

load pima.txt;

disp('Part b')
[normalized, mu, sigma] = normalize(pima(:,3));
disp(normalized(1:5))

disp('Part c')
discrete = discretize_attribute(pima(:,3),10);
disp(discrete(1:5))
