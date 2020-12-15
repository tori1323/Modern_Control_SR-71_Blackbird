function [gains,table,sys] = PI_NZSP(sys_temp,pacing,table,t_final,ym,dis)
% NZSP - Input a continuous system with specified gains to meet design 
% requirements.
% Inputs:
%   - sys_temp: Contains the State Space Values and Gains
%   - pacing: Contains the period, T and the value h, the discritization
%             value
%   - table: Maintains all x, u, and t values
%   - t_final: The final time within the simulation
%   - ym: Where driving control to
%   - dis: Disturbance
% Outputs:
%   - gains: Contain K, Q_hat, R_hat, M, S (P_ss = P(N), E (eigenvalues)
%   - sys: New system with PI added
%   - table: Updated table with new values from simulation

% Begin Code::
    
    N = length(sys_temp.A);
    
    % Preliminaries
    [sys_old,sys,gains] = Build_Discrete_System_PI(sys_temp.A,sys_temp.B,sys_temp.C,...
        sys_temp.D,sys_temp.Q,sys_temp.R,pacing.h,pacing.T,sys_temp.H); % Build System
    
    % New System for PI NZSP
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    H = sys_temp.H;
    
    % From old way of doing NZSP
    A1 = sys_old.A;
    B1 = sys_old.B;
    C1 = sys_old.C;
    D1 = sys_old.D;
    
    % Pull out times
    T = pacing.T; 
    h = pacing.h;
    
    Y = [zeros(length(A)-1,1);-h]; % Goes into x equation

    % Initialize variables 
    frames = 0:T:t_final;
    
    % Create points for a deflection surface
    slope = ym/4; % 1.9 comes from time requirement
    table.track = zeros(1,length(table.t)); % Initialize variable
    for j = 1:(length(table.t)-1)
        if j < 401
            table.track(1,j+1) = table.track(1,j) + slope*pacing.h;
        else
            table.track(1,j+1) = ym;
        end
    end
    
    % Solve for pi22 and pi12
    [pi12,pi22] = QPMCALC(A1-eye(size(A1)),B1(:,2),H,0);
    table.pi12 = pi12;
    table.pi22 = pi22;
    
    % Initial Variables
    table.u(:,1) = (pi22 + gains.K(:,(1:N))*pi12)*ym - gains.K * table.x(:,1); % Update control values
    table.y(:,1) = C * table.x(:,1) + D * table.u(:,1); % Update output values
    
    % Begin Loop over Values
    for i = 1:(length(table.t)-1)
        
        table.x(:,i + 1) = A * table.x(:,i) + B * table.u(:,i) + ...
            (Y*ym) + dis * h; % Update state values
        
        if sum(ismember(table.t(1,i+1),frames)) > 0
            table.u(:,i+1) = (pi22 + gains.K(:,(1:N))*pi12)*ym - gains.K * table.x(:,i+1); % Update control values
        else
            table.u(:,i+1) = table.u(:,i); % Update control value for ZOH
        end
        
        table.y(:,i+1) = C * table.x(:,i+1) + D * table.u(:,i+1); % Update output values
    end
    
    % Update the last values
    table.u(:,length(table.t)) = (pi22 + gains.K(:,(1:N))*pi12)*ym - gains.K * table.x(:,end); % Update control values
    table.y(:,length(table.t)) = C * table.x(:,end) + D * table.u(:,end); % Update output values
end