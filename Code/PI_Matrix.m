function [sys2] = PI_Matrix(sys,H,T)
% Solves for the gains and discrete system of the continuous system
% Inputs:
%   - sys: System
%   - H: Integrated State
%   - T: Period of Sample Data
% Outputs:
%   - sys2: Contains the discrete version of the continous system


% Begin Code ::

% Build Systems
[n,~] = size(sys.A);
[~,b] = size(sys.B);
[c,~] = size(H);
[e,f] = size(sys.C);
[~,k] = size(sys.D);

A = [sys.A zeros(n,c);
    (H * T) eye(c)];
B = [sys.B;
    zeros(c,b)];

C = [sys.C zeros(e,c); zeros(c,(f+c))];
C(e+c,f+c) = 1;
D = [sys.D; zeros(c,k)]; 
sys2 = ss(A,B,C,D);

end
