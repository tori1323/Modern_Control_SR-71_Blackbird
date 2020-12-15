function [sys2] = PIF_CRW_Matrix(sys,H,T)
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

A = [sys.A,     sys.B,   zeros(n,c); 
    zeros(b,n), eye(b),  zeros(b,c);
       T*H,     zeros(c,b), eye(c)];
  
B = [zeros(n,b);
       T*eye(b);
      zeros(c,b)];


C = [eye(n+b+c)];
D = [zeros(c+b+n,k)]; 
sys2 = ss(A,B,C,D);
sys2.Ts = T;

end
