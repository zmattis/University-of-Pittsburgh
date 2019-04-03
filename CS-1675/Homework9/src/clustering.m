% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 9
%  * April 2, 2019
%  *
%  * This Matlab file is a function that runs
%  * the kmeans algorithm given an input data
%  * file and an integer k. It categorizes
%  * each data point into 1 of k categories
%  * and plots each point with a different color.
%  */

function[] = clustering(data_src, k)

cluster_data = load(data_src);
min_cluster = realmax;
min_cluster_group = zeros(1,k);

for iters=1:30
  [id, foo, fbar] = kmeans(cluster_data, k);

  dsize = size(id, 1);
  psize = size(cluster_data, 2);
  group = zeros(k, psize);
  members = zeros(1,k);

  for i=1:dsize
    members(id(i)) = members(id(i)) + 1;
    for j=1:psize
      group(id(i), j) = group(id(i), j) + cluster_data(id(i), j);
    end
  end

  if min_cluster > sum(fbar)
    min_cluster = sum(fbar);
    for c=1:k
      min_cluster_group(c) = members(c);
    end
  end
  %disp(members);
  for i=1:dsize
    for j=1:psize
      group(id(i), j) = group(id(i), j) / members(id(i));
    end
  end
end

x = cluster_data(:,1);
y = cluster_data(:,2);
colors = ['r','g','b','c','m','y'];

figure();
title(strcat('Clustering Data, k=', num2str(k)));
xlabel('x');
ylabel('y');
hold on;
% data points (x_i)
for i=1:k
  scatter( x(id == i), y(id ==i), colors(i) );
end
% means (u)
scatter( foo(:,1), foo(:,2), 50, 'o', 'black', 'filled' );

disp(min_cluster);
disp(min_cluster_group);
disp(foo);
