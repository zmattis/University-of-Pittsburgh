% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 3
%  * February 6, 2019
%  *
%  * This Matlab file performs ML analysis of
%  * a Beta distribution on a given
%  * input file, coin.txt.
%  */

load coin.txt;


disp('Part a')
theta = sum(coin(:) == 1);
total = theta + sum(coin(:) == 0);

ml_estimate = theta/total


disp('Part b/c')

prior = [1 1];
figure();
hold on;

plot([0:.01:1], betapdf([0:0.01:1], prior(1), prior(2)));

prior(1) = prior(1) + theta;
prior(2) = prior(2) + (total-theta);
map_estimate = (prior(1) - 1)/(prior(1) + prior(2) - 2)

plot([0:.01:1], betapdf([0:0.01:1], prior(1), prior(2)));
plot([map_estimate map_estimate], [0 max(betapdf([0:0.01:1], prior(1), prior(2)))]);

title('Beta(\theta | 1,1)');
xlabel('\theta');
ylabel('Density');
legend('Prior','Posterior','MAP Estimate');

pause;
hold off;

disp('Part d')

prior = [4 2];
figure();
hold on;

plot([0:.01:1], betapdf([0:0.01:1], prior(1), prior(2)));

prior(1) = prior(1) + theta;
prior(2) = prior(2) + (total-theta);
map_estimate = (prior(1) - 1)/(prior(1) + prior(2) - 2)

plot([0:.01:1], betapdf([0:0.01:1], prior(1), prior(2)));
plot([map_estimate map_estimate], [0 max(betapdf([0:0.01:1], prior(1), prior(2)))]);

title('Beta(\theta | 4,2)');
xlabel('\theta');
ylabel('Density');
legend('Prior','Posterior','MAP Estimate');


pause;
close all;