function [gains,table,sys] = PI_SDR(sys_temp,pacing,table,t_final,dis)
% Sample Data Regulator - Input a continuous system with specified gains to
% meet design requirements.
% Inputs:
%   - sys_temp: Contains the State Space Values and Gains
%   - pacing: Contains the period, T and the value h, the discritization
%             value
%   - table: Maintains all x, u, and t values
%   - t_final: The final time within the simulation
%   - dis: Disturbance
% Outputs:
%   - gains: Contain K, Q_hat, R_hat, M, S (P_ss = P(N), E (eigenvalues)
%   - table: Updated table with new values from simulation
%   - sys: New system with PI added

% Begin Code::
    
    % Preliminaries
    [sys_old,sys,gains] = Build_Discrete_System_PI(sys_temp.A,sys_temp.B,sys_temp.C,...
        sys_temp.D,sys_temp.Q,sys_temp.R,pacing.h,pacing.T,sys_temp.H); % Build System
    
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    
    T = pacing.T; % Period of sampling
    h = pacing.h;

    % Initialize variables 
    frames = 0:T:t_final;
    
    % Initial Variables
    table.u(:,1) = - gains.K * table.x(:,1); % Update control values
    table.y(:,1) = C * table.x(:,1) + D * table.u(:,1); % Update output values
    
    % Begin Loop over Values
    for i = 1:(length(table.t)-1)
        
        table.x(:,i + 1) = A * table.x(:,i) + B * table.u(:,i) + dis * h; % Update state values
        
        if sum(ismember(table.t(1,i+1),frames)) > 0
            table.u(:,i+1) = - gains.K * table.x(:,i+1); % Update control values
        else
            table.u(:,i+1) = table.u(:,i); % Update control value for ZOH
        end
        
        table.y(:,i+1) = C * table.x(:,i+1) + D * table.u(:,i+1); % Update output values
    end
    
    % Update the last values
    table.u(:,length(table.t)) = - gains.K * table.x(:,end); % Update control values
    table.y(:,length(table.t)) = C * table.x(:,end) + D * table.u(:,end); % Update output values
end