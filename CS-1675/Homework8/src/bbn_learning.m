% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 8
%  * March 25, 2019
%  *
%  * This Matlab file is used to calculate the
%  * parameters of a Bayesian Belief Network,
%  * given whether a patient has pneumonia
%  * or not from the file 'pneumonia.tex'.
%  * The BBN parameters include:
%  *     - Fever
%  *     - Paleness
%  *     - Cough
%  *     - HighWBCcount
%  */

load pneumonia.tex;
data = pneumonia;

% Fever
fever1 = data(data(:,5) == 1,1:4);
fever_p_t = sum(fever1(:,1) == 1)/size(fever1,1);
fever0 = data(data(:,5) == 0,1:4);
fever_p_f = sum(fever0(:,1) == 1)/size(fever0,1);

% Paleness
pale1 = data(data(:,5) == 1,1:4);
pale_p_t = sum(pale1(:,2) == 1)/size(pale1,1);
pale0 = data(data(:,5) == 0,1:4);
pale_p_f = sum(pale0(:,2) == 1)/size(pale0,1);

% Cough
cough1 = data(data(:,5) == 1,1:4);
cough_p_t = sum(cough1(:,3) == 1)/size(cough1,1);
cough0 = data(data(:,5) == 0,1:4);
cough_p_f = sum(cough0(:,3) == 1)/size(cough0,1);

% HighWBCcount
wbc1 = data(data(:,5) == 1,1:4);
wbc_p_t = sum(wbc1(:,4) == 1)/size(wbc1,1);
wbc0 = data(data(:,5) == 0,1:4);
wbc_p_f = sum(wbc0(:,4) == 1)/size(wbc0,1);
