% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 10
%  * April 9, 2019
%  *
%  * This Matlab file is a function that runs
%  * the learning classification via a
%  * default tree such that any parent node
%  * has at least 10 examples and nodes are
%  * split till the leafs are not pure.
%  */

function [test_y]  = DT_base_full(tr_x, tr_y, test_x, params)

    tree = fitctree(tr_x,tr_y,'splitcriterion','gdi','Prune','off');
    test_y = predict(tree,test_x);
    return;

end