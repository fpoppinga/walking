%% FLIPM

% parameters
k = 1;
b = 1;
m1 = 1;
m2 = 1e-3;

% model
% let x = [c1 c1' c2 c2']^T

A = [0      1       0       0
     -k/m1  -b/m1   k/m1    b/m1
     0      0       0       1
     k/m2   b/m2    -k/m2   -b/m2];
 
b = [0      0       0       1/m2]';
 
C = [1      0       0       0
     0      0       1       0];
 
sys = ss(A, b, C, 0);

%%

