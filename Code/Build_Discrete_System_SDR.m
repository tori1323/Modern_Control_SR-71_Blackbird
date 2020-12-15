function [sys2,gains] = Build_Discrete_System_SDR(A,B,C,D,Q,R,h,T)
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
% Outputs:
%   - sys2: Contains the discrete version of the continous system
%   - gains: K, Q_hat, R_hat, M, S (P_ss = P(N), E (eigenvalues)

% Begin Code ::

% Build Systems
sys = ss(A,B,C,D);
[sys2,~] = c2d(sys,h);
[gains.K, gains.Q_hat, gains.R_hat, gains.M, gains.S, gains.E] = lqrdjv(A,B,Q,R,T);
end
