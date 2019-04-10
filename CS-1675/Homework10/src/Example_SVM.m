%%% uses bagging to learn 5 SVM model from tr_x and tr_y
%%% and predicts the labels for test_x
%%% please note you can pass additional parameters to the SVML_base model
%%% if needed which would replace []

[test_y] = Bag_classifier(tr_x,tr_y,test_x,'[@SVML_base,5,[]]');
