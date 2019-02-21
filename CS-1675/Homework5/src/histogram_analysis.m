%% Problem 2(f) - plots histogram with 20 bins of vector X of attribute values
function [] = histogram_analysis(x, x_label, y_label)

hist(x,20);
xlabel(x_label);
ylabel(y_label);
