% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 4
%  * February 13, 2019
%  *
%  * This Matlab file is a procdure
%  * used to simulate testing of the
%  * gradient methodology.
%  */


for i=1:train_trials    %length(train_n)
    ind = mod(i,length(train_n(:,:)))+1;
    alpha = 2/i;
    fxw = LR_predict(train_n(ind,1:13),w);
    w = (w' + alpha*(train_n(ind,14) - fxw)*train_n(ind,1:13))';
    if mod(i, 50) == 0
        y_test  = LR_predict(test_n(:,1:13), w);
        y_train = LR_predict(train_n(:,1:13),w);
        mse_train = sum((y_train - train_n(:,14)).^2)/length(train_n(:,14));
        mse_test  = sum((y_test - test_n(:,14)).^2)/length(test_n(:,14));
        %g = add_to_progress_graph(g, i, mse_train, mse_test);
    end
end