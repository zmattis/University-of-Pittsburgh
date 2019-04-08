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
%  * AUROC scores.
%  */

data = load('FeatureSelectionData.txt');

X = data(:,1:70);
y = data(:,71);

auroc = zeros(70,2);
auroc(:,2) = 1:70;

for i=1:70
    auroc(i) = AUROC_score(X(:,i),y);
end

auroc(auroc < 0.5) = 1-auroc(auroc < 0.5);
auroc = sortrows(auroc,1,'descend');
top_20 = auroc(1:20,:);
