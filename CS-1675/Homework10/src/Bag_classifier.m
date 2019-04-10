% Classify using the Bagging algorithm
% *** Inputs:
% 	train_patterns - Train patterns
%   train_targets  - Train targets
%  	test_patterns  - Test patterns
%  	params:
%  	base_classifier       - base classifier
%  	NumberOfIterations    - number of models
% 	Classifier_parameters - additional parameters for the base classifier
%
% *** Outputs:
%  test_y - Predicted targets
%  E      - Errors through the iterations

function [test_y, E] = Bag_classifier(tr_x, tr_y, test_x, params)

[base_classifier, k_max, alg_params] = process_params(params);

[M,N]     = size(tr_x);
W         = ones(M,1)/M;
IterDisp  = 10;
Nc        = length(unique(tr_y));

test_y    = zeros(size(test_x,1),1);

% Bagging loop with bootstrap sampling
for k = 1:k_max,
    % Train weak learner Ck using the data sampled according to W:
    %...so sample the data according to W
    randnum = rand(M,1);
    cW      = cumsum(W);
    indices = zeros(M,1);
    for i = 1:M,
        %Find which bin the random number falls into
        loc = max(find(randnum(i) > cW))+1;
        if isempty(loc)
            indices(i) = 1;
        else
            indices(i) = loc;
        end
    end

    %...and now train the classifier and run it on the test set
    test_single = feval(base_classifier, tr_x(indices,:), tr_y(indices), test_x, alg_params);

    %Update the test targets
    test_y = test_y + test_single;

end

%%% do the binary choice
test_y =(test_y./k_max) > 0.5;
end
