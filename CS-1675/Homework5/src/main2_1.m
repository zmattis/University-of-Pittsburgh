% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 5
%  * February 21, 2019
%  *
%  * This Matlab file splits the dataset
%  * pima.txt into two classes, defined
%  * by attribute 9, the binary "class"
%  * variable.
%  */

load pima.txt;

class_zero = pima(pima(:,9) == 0, :);
class_one = pima(pima(:,9) == 1, :);

% class 0
figure();
title('Class 0');
for i=1:8
  subplot(2,4,i);
  histogram_analysis( class_zero(:,i), strcat('Attr. ', num2str(i)), '' );
end

% class 1
figure();
title('Class 1');
for i=1:8
  subplot(2,4,i);
  histogram_analysis( class_one(:,i), strcat('Attr. ', num2str(i)), '' );
end

