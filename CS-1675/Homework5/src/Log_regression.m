%% Logistic regression using batch gradient descend
% inputs: 1. the input data
%         2. the output data
%         3. number of steps
% returns: final weights
%%% annealed learning: 2/sqrt(k)

function W = Log_regression(X, Y, K)

col_ones = ones(size(X, 1), 1);
X = horzcat(col_ones, X);           % add a column of ones on the left to X
W = ones(size(X, 2), 1);            % initialize W to 1 to start with 

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
end