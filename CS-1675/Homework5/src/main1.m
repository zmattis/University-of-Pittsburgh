% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 5
%  * February 21, 2019
%  *
%  * This Matlab file is used to
%  * normalize datafiles and write
%  * them to new .txt files.
%  */

load pima_test_norm.txt;
load pima_train_norm.txt;
load pima_train.txt;
load pima_test.txt;

graph = init_progress_graph();
graph.pause = 0;

X = pima_train_norm(:,1:8);
Y = pima_train(:,9);
K = 500;

col_ones = ones(size(X, 1), 1);
X = horzcat(col_ones, X);           % add a column of ones on the left to X
W = ones(size(X, 2), 1);            % initialize W to 1 to start with 

X_test = pima_test_norm(:,1:8);
col_ones = ones(size(X_test, 1), 1);
X_test = horzcat(col_ones, X_test);           % add a column of ones on the left to X

for k = 1:1:K                       %%% number of steps
    sum_err = 0;                    %%% initialize batch error function gradient
    for row = 1:1:size(X, 1)
        x = X(row,:)';
        y = Y(row,:);
        f = 1/(1 + exp(-(W'*x)));
        err = (y - f) * x;          % error (on-line gradient)
        sum_err = sum_err + err;    % update batch error function gradient
    end
    alpha = 2/sqrt(k);
    W = W + (alpha * sum_err);
    
    if mod(k,50) == 0
        % train errors
        [TP, TN, FP, FN] = errors(X, W, Y);

        SENS = TP/(TP + FN);
        SPEC = TN/(TN + FP);
        mis = (FP + FN)/(TP + TN);


        % test errors

        Y_test = pima_test(:,9);

        %[TP, TN, FP, FN] = errors(X_test, W, Y_test);

        SENS_test = TP/(TP + FN);
        SPEC_test = TN/(TN + FP);
        mis_test = (FP + FN)/(TP + TN);
        
        graph = add_to_progress_graph(graph, k, mis, mis_test);
        
    end
end


%------- errors for train

[TP, TN, FP, FN] = errors(X, W, Y)

SENS = TP/(TP + FN)
SPEC = TN/(TN + FP)
mis = (FP + FN)/(TP + TN)


%------- errors for test

X_test = pima_test_norm(:,1:8);
col_ones = ones(size(X_test, 1), 1);
X_test = horzcat(col_ones, X_test);           % add a column of ones on the left to X
Y_test = pima_test(:,9);

[TP, TN, FP, FN] = errors(X_test, W, Y_test)

SENS_test = TP/(TP + FN)
SPEC_test = TN/(TN + FP)
mis_test = (FP + FN)/(TP + TN)