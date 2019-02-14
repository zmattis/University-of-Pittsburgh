
%% graphs the progress of the mean train and test errors for iterative procedures
% arguments: 1. graph structure
%            2. step number
%            3. train error
%            4. test error
% returns: updated graph structure

function [pgraph] = add_to_progress_graph(pgraph, step, traine, teste)

pgraph.step=[pgraph.step step];
pgraph.train=[pgraph.train traine];
pgraph.test=[pgraph.test teste];
plot(pgraph.step,pgraph.train,'-b');
hold on;
plot(pgraph.step,pgraph.test,'--r');
xlabel('Step');
ylabel('Mean squared error');
title('(Progress of mean errors)') %title
legend('Train set','Test set') %label of the graph
pause(pgraph.pause);
hold off;
end
