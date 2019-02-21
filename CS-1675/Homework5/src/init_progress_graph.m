%% graphs the progress of the mean train and test errors for iterative procedures
% initialization of the progress graph

function [pgraph] = init_progress_graph
pgraph.step=[];
pgraph.train=[];
pgraph.test=[];
pgraph.pause=1.5;
%% starts the new figure
figure;
end
