% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 10
%  * April 8, 2019
%  *
%  * This Matlab file is a driver file used
%  * to calculate the predictive power of
%  * the dimensionality of a dataset,
%  * 'FeatureSelectionData.txt' via
%  * Fisher scores.
%  */

data = load('FeatureSelectionData.txt');

X = data(:,1:70);
y = data(:,71);

fisher = zeros(70,2);
fisher(:,2) = 1:70;

for i=1:70
    fisher(i) = Fisher_score(X(:,i),y);
end

fisher = sortrows(fisher,1,'descend');
top_20 = fisher(1:20,:);
