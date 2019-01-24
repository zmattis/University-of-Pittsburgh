% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 1
%  * January 23, 2019
%  *
%  * This Matlab file performs data analysis on a given
%  * input file, pima.txt.
%  */

load pima.txt;

disp('Part a')
maxs = max(pima,[],1)
mins = min(pima,[],1)
ranges = [maxs(1:8); mins(1:8)];
pause;

disp('Part b')
means = mean(pima,1);
means = means(1:8)
variance = var(pima,1);
variance = variance(1:8)
pause;

disp('Part c')
ind0 = pima(:,9) == 0;
ind1 = pima(:,9) == 1;
class0 = pima(ind0,:);
class1 = pima(ind1,:);
mean0 = mean(class0)
std0 = std(class0)
mean1 = mean(class1)
std1 = std(class1)
pause;

disp('Part e')
for i=1:7
    histogram_analysis(pima(:,i))
    figure()
end
histogram_analysis(pima(:,i+1))
pause;
close all;

disp('Part f')
ind0 = pima(:,9) == 0;
ind1 = pima(:,9) == 1;
class0 = pima(ind0,:);
class1 = pima(ind1,:);

for i=1:8
    histogram_analysis(class0(:,i))
    xlabel(i)
    pause;
end
for i=1:8
    histogram_analysis(class1(:,i))
    xlabel(i)
    pause;
end

disp('Part g')
for i=1:8
    for j=i+1:8
        scatter_plot(horzcat(pima(:,i),pima(:,j)))
        xlabel(i)
        ylabel(j)
        pause;
    end
end
close all;

