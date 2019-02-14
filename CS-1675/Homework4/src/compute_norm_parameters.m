%%%%%% Computes normalizers from x matrix 

function [mean_norm std_norm]=compute_norm_parameters(x)

mean_norm = mean(x);
std_norm  = std(x);
