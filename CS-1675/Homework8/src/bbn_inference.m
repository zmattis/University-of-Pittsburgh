% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 8
%  * March 25, 2019
%  *
%  * This Matlab file is used to determine the
%  * the likelyhood of having pneumonia,
%  * given a set of symptoms. The parameters
%  * were calculated via the learning.m file.
%  */

f = [1.0,1.0,1.0;1.0,.4,.6;1.0,.1,.9];
p = [1.0,1.0,1.0;1.0,.5,.5;1.0,.3,.7];
c = [1.0,1.0,1.0;1.0,.9,.1;1.0,.1,.9];
h = [1.0,1.0,1.0;1.0,.5,.5;1.0,.2,.8];

dist(1,:,:) = f;
dist(2,:,:) = p;
dist(3,:,:) = c;
dist(4,:,:) = h;

f = .606;
p = .884;
c = .496;
h = .506;

pneu = 0.02;

dist2 = [1,1,1,1;1-f,1-p,1-c,1-h;f,p,c,h];

load example.txt;

for i=1:size(example,1)
    n = diag(dist(:,example(i,:)+2,3));
    d = diag(dist2(example(i,:)+2,:));
    chance = pneu * prod(n) ./ prod(d);
    fprintf('Instance %d with params %d, %d, %d, %d has a %f chance of having pneuonia.\n',i,example(i,1),example(i,2),example(i,3),example(i,4),chance);

end