% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 5
%  * February 21, 2019
%  *
%  * This Matlab file is a function that predicts class
%  * labels based on the class posterior.
%  */

function [ g0, g1, p_0, p_1 ] = predict_NB( x, exp_mu, norm_mu, norm_sigma )

exp_a = [1 5 7 8];
norm_a = [2 3 4 6];
p_0 = zeros(8,1);
p_1 = zeros(8,1);

for i=exp_a
    p_0(i) = p_exp(x(i), exp_mu(i,1));
    p_1(i) = p_exp(x(i), exp_mu(i,2));
end
for i=norm_a
    p_0(i) = p_norm(x(i), norm_mu(i,1), norm_sigma(i,1));
    p_1(i) = p_norm(x(i), norm_mu(i,2), norm_sigma(i,2));
end
g0 = sum(log(p_0));
g1 = sum(log(p_1));

end

function [ prob ] = p_exp( x, mu )
   
prob = 1/mu * exp(-x/mu);

end

function [ prob ] = p_norm( x, mu, sigma )

prob = 1/(sigma*sqrt(x*pi))*exp(-(x-mu)^2/(2*sigma^2));

end