function [sys2,sys3,gains] = Build_Discrete_System_PIF_CRW(A,B,C,D,Q,R,h,T,H)
% Solves for the gains and discrete system of the continuous system
% Inputs:
%   - A: State Matrix
%   - B: Control Matrix
%   - C: Output Matrix
%   - D: Direct Transition Matrix
%   - Q: State Gain
%   - R: Control Gain
%   - h: Discritizing Rate
%   - T: Period of Sample Data
%   - H: Integrated State
% Outputs:
%   - sys2: Contains the discrete version of the continous system
%   - sys3: PI added to discrete system
%   - gains: K, Q_hat, R_hat, M, S (P_ss = P(N), E (eigenvalues)

% Begin Code ::

[n,~] = size(A);
[~,b] = size(B);
[c,~] = size(H);

% Build Systems
sys = ss(A,B,C,D);

% Continuous System
A1 = [A, B, zeros(n,c); 
      zeros(b,n+b+c);
      H, zeros(c,b+c)];
  
B1 = [zeros(n,b);
       eye(b);
      zeros(c,b)];

% Discrete
[sys2,~] = c2d(sys,h);
[sys3] = PIF_CRW_Matrix(sys2,H,h);
[sys4] = d2c(sys3);
[gains.K, gains.Q_hat, gains.R_hat, gains.M, gains.S, gains.E] = lqrdjv(A1,B1,Q,R,T);
end
