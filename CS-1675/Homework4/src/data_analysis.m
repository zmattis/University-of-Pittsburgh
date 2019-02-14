% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 4
%  * February 13, 2019
%  *
%  * This Matlab file performs data analysis of
%  * given input file, housing.txt.
%  */

load housing.txt;


disp('Part b')
corr = corrcoef(housing);
corr_target = corr(:,14)

[max_corr, max_corr_idx] = max(corr_target(1:13))
[min_corr, min_corr_idx] = min(corr_target(1:13))

disp('Part c')
for i=1:13
    figure();
    scatter(housing(:,i), housing(:,14));
    pause;
end

pause;
close all;


disp('Part d')
corrs = corrcoef(housing);
corrs(corrs >= 1.0) = 0.0;

corrs = tril(corrs);
corrs = abs(corrs);
[t, ind_x] = max(corrs)
[m, ind_y] = max(t)


for i=1:14
    for j=i+1:14
        housing_scatter = horzcat(housing(:,i), housing(:,j));
        scatter(housing_scatter(:,1), housing_scatter(:,2));
        xlabel(i)
        ylabel(j)
        pause;
    end
end

close all;