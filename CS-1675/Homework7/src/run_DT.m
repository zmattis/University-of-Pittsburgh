%%% learns the decision tree 
%%% *************************************************************
%%% Milos Hauskrecht
%%% 2750 Machine Learning, University of Pittsburgh
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% load the train and test data (both are normalized)
load pima_train.txt;
load pima_test.txt;
tr_data = pima_train;
test_data = pima_test;

data_col= size(tr_data,2)
n_features = data_col - 1;

%%% create x
x = tr_data(:,1:n_features);
%% create y vector
y=tr_data(:,data_col);
 
%% builds x for the the test set
x_test = test_data(:,1:n_features);
%% builds y vector for the test set
y_test=test_data(:,data_col);

%%% classification: default tree
%%% learn a classification tree, such that any parent node 
%%%% has at least 10 examples (default) and nodes are split till the leafs
%%% are not pure, gini criterion is used for splits
tree=fitctree(x,y, 'splitcriterion','gdi');
%%% shows the tree logic
view(tree);
%%% show the graphics of the tree
view(tree,'Mode','graph');

%%% Predicts the label on the test set
'Test error:' 
y_pred=predict(tree,x_test);
error= sum(abs(y_pred-y_test))/size(y_test,1)


%%%% new tree with restrictions on the tree size, parent size, and leaf sizes 
%%%% learns a tree with at most 25 splits, such that any parent node 
%%%% has at least 10 examples and any leaf has at least 2 examples
new_tree=fitctree(x,y,'MaxNumSplits',25, 'MinParentSize',10,'MinLeafSize',2,'splitcriterion','gdi');
%%% show the tree logic
view(new_tree);
%%% show the graphics of the tree
view(new_tree,'Mode','graph');
%%% evaluate the tree on the test data
y_pred=predict(new_tree,x_test);
'Test error (new tree):' 
error= sum(abs(y_pred-y_test))/size(y_test,1)

