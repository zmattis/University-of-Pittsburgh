% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 3
%  * February 6, 2019
%  *
%  * This Matlab file performs ML analysis of
%  * a Gaussian distribution on a given
%  * input file, gaussian.txt.
%  */

load gaussian.txt


disp('Part a')
scatter( gaussian(:,1), gaussian(:,2) )
means = sum(gaussian)/length(gaussian)
hold on;
hold off;
pause;


disp('Part b')
diff = bsxfun(@minus, gaussian, means);
covars = cov(gaussian(:,1), gaussian(:,2))

c1 = covars(1,1); 
c2 = covars(2,2);
m1 = means(1);
m2 = means(2);

divs = 13;
s = 2;
x1 = [m1-c1*s:(m1+c1*s - (m1-c1*s))/2 /20:m1+c1*s];
x2 = [m2-c2*s:(m2+c2*s - (m2-c2*s))/2 /20:m2+c2*s];

[X1, X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)], means, covars);
F = reshape(F, length(x1), length(x2));
surf(X1, X2, F);
hold on;
axis([min(x1) max(x1) min(x2) max(x2) min(F(:)) max(F(:))]);
pause;
hold off;

disp('Part c')
d1 = normpdf(x1, m1, c1);
d2 = normpdf(x2, m2, c2);

plot(x1, d1, 'b');
hold on;
plot(x2, d2, 'r');

plot([m1 m1], [min(d1) max(d1)], '--b');
plot([m2 m2], [min(d2) max(d2)], '--r');

pause; 

close all;