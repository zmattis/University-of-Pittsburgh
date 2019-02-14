%%% Applies normalization to x matrix (attributes are in columns)

function [x_norm]=normalize(x,mean_norm,std_norm)

x_norm=zeros(size(x,1),size(x,2));
for j = 1:size(x,2)
    x_norm(:,j) = (x(:,j) - mean_norm(j))/std_norm(j);
end
