% /**
%  * @author Zachary M. Mattis
%  * CS 1675
%  * Assignment 1
%  * January 23, 2019
%  *
%  * This Matlab file performs basic matrix operations
%  * on preconfigured matrices.
%  */

A = [ 1, 2, 5; 3, 4, 6];
B = [ 7, 1, 9; 2, 2, 3; 4, 8, 6];
C = [ 8, 6, 5; 1, -3, 4; -1, -2, 4];

%A
disp('A^T = ')
disp(transpose(A))

%B
disp('B^-1')
disp(mpower(B,-1))

%C
disp('B + C')
disp(B + C)

%D
disp('B - C')
disp(B - C)

%E
disp('A * B')
disp(A * B)

%F
disp('B * C')
disp(B * C)

%G
disp('B * A')
%disp(B * A)
disp('ERROR: Bad inner dimensions')