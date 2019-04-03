% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 9
%  * April 2, 2019
%  *
%  * This Matlab file uses Matlab's
%  * built-in linkage() and cluster()
%  * functions to hierarchically cluster
%  * a dataset, 'clustering_data.txt'.
%  */

Y = load('clustering_data.txt');

% part a
Z = linkage( Y, 'complete', 'euclidean' );
dendrogram(Z,0);

% part b
k=4;
C = cluster( Z, 'maxclust', k );

x = Y(:,1);
y = Y(:,2);
colors = ['r','g','b','c','m','y'];

% scatter plot
figure();
title(strcat('Hierarchical Clustering, k=', num2str(k)));
xlabel('x');
ylabel('y');
hold on;
% data points
for i=1:k
  scatter( x(C == i), y(C ==i), colors(i) );
end
