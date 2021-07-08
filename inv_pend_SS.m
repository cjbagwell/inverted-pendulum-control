% JORDAN BAGWELL
% JEFFREY MAYS
% NICHOLAS SCIORTINO

function [A, B, C, D] = inv_pend_SS(m1, m2, L, b, g)

den = (4*m1+m2)*(m1+m2);

A = [0      1                   0                           0;
     0      -4*(m1+m2)*b/den    3*(m2^2+m1*m2)*g/den        0;
     0      0                   0                           1;
     0      -6*b/(L*(4*m1+m2))  6*(m1+m2)*g/(L*(4*m1+m2))   0];
 
B = [0;
     4*(m1+m2)/den;
     0;
     6/(L*(4*m1+m2))];
 
C = eye(4);
D = [0;0;0;0];
% C = [1 0 0 0;
%      0 0 1 0];
%  
% D = [0;
%      0];

 