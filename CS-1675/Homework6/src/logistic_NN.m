%%% learns a simple NN network for classification with one (output) unit 
%%% with sigmoidal function
%%% *************************************************************
%%% Milos Hauskrecht
%%% CS 2750 Machine Learning, University of Pittsburgh
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% load the train and test data (both are normalized)
load pima_train.txt;
load pima_test.txt;
tr_data = pima_train;
test_data = pima_test;

data_col= size(tr_data,2);
n_features = data_col - 1;

%%% create x
x = tr_data(:,1:n_features);
%% create y vector
y=tr_data(:,data_col);
 
%% builds x for the the test set
x_test = test_data(:,1:n_features);
%% builds y vector for the test set
y_test=test_data(:,data_col);

%%% builds a one layer neural network (no hidden units) with a sigmoidal output function
%%% to be trained with the gradient method
net=patternnet([]);
view(net)

%% sets the parameters of the NN model
%% see the NN toolbox documentation

%%% use conjugate gradient to train the model
net.trainFcn='trainscg';

net.trainParam.epochs = 2000;
net.trainParam.show = 10;
net.trainParam.max_fail=1500;
net.trainParam.min_grad=1e-10;

%%% training of the neural net
[net, tr] = train(net,x',y') 

%% runs learned network on inputs in x (training set)
res=net(x');
%%% mean classification error on the training data
class_error_train=sum(abs(y-round(res)'))/size(res,2)
%%% 'Mean squared error (mse) on the training data'
mse_error_train = perform(net,y',res)


%% runs learned network on inputs in x (testing set)
res_test = net(x_test');
%%% mean classification error on the testing data
class_error_test=sum(abs(y_test-round(res_test)'))/size(res_test,2)
%%% 'Mean squared error (mse) on the testing data'
mse_error_test = perform(net,y_test',res_test)

%%% weights
cell2mat(net.IW)
%%% bias
cell2mat(net.b)


