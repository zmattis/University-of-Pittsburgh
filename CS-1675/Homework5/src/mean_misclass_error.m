%% takes vectors of the same size, compares them and 
function [mean]=mean_misclass_error(a,b)

n=length(a);
mean=sum(abs(a-b))/n

return;