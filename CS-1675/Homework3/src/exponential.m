% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 3
%  * February 6, 2019
%  *
%  * This Matlab file plots an exponential distribution
%  * of the form y=f(x|b)=(1/b)*e^(-x/b)
%  */


x = [0:0.1:2];
b1 = 1;
b2 = 0.25;
b3 = 4;

y1 = exppdf(x, b1);
y2 = exppdf(x, b2);
y3 = exppdf(x, b3);

hold on;
plot(x, y1, '-b');
plot(x, y2, '-r');
plot(x, y3, '-g');
title('Exponential Distribution');
xlabel('x');
ylabel('p(x|b)');
legend('b = 1','b = 0.25','b = 4');

hold off;
pause;