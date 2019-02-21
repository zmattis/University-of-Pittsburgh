% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 5
%  * February 21, 2019
%  *
%  * This Matlab file calculates the parameters
%  * for both normal and exponential distributions
%  * of the pima.txt dataset.
%  */

load pima.txt;

class_0 = pima(pima(:,9) == 0, :);
class_1 = pima(pima(:,9) == 1, :);

% exponential
exp_0_muhat = zeros(4, 1);
exp_0_muci = zeros(4,2);
exp_1_muhat = zeros(4, 1);
exp_1_muci = zeros(4,2);

for c=[1 5 7 8]
    [muhat, muci] = expfit(class_0(:,c));
    exp_0_muhat(c) = muhat;
    exp_0_muci(c,:) = muci;
    
    [muhat, muci] = expfit(class_1(:,c));
    exp_1_muhat(c) = muhat;
    exp_1_muci(c,:) = muci;
end

% normal
norm_0_mu = zeros(4, 1);
norm_0_sigma = zeros(4,1);
norm_0_muci = zeros(4,2);
norm_0_sci = zeros(4,2);
norm_1_mu = zeros(4, 1);
norm_1_sigma = zeros(4,1);
norm_1_muci = zeros(4,2);
norm_1_sci = zeros(4,2);
for c=[2 3 4 6]
    [norm_0_mu(c),norm_0_sigma(c),norm_0_muci(c,:),norm_0_sci(c,:)] = normfit(class_0(:,c));
    [norm_1_mu(c),norm_1_sigma(c),norm_1_muci(c,:),norm_1_sci(c,:)] = normfit(class_1(:,c));
end