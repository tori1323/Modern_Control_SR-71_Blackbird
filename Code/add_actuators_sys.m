function [A_new,B_new,C_new,D_new] = add_actuators_sys(A,B,C,D)
% Adds actuator dynamics to the overall system.
% Inputs:
%   - A: State Matrix
%   - B: Input Matrix
%   - C: Output Matrix
%   - D: Feedthrough Matrix
% Outputs:
%   - A_new: New State Matrix w/Actuator dynamics added
%   - B_new: New Input Matrix w/Actuator dynamics added
%   - C_new: New Output Matrix w/Actuator dynamics added
%   - D_new: New Feedthrough Matrix w/Actuator dynamics added

% Begin Code::
    % Define Actuaor Matrix
    Act = [10 0;
           0 10];
    fill = zeros(2,5); % Fill left side matrix augmentation

    A_new = [A B; fill -Act];
    B_new = [zeros(5,2);Act];
    C_new = [1   0   0   0   0   0   0     % beta             % States
             0   1   0   0   0   0   0     % p
             0   0   1   0   0   0   0     % r
             0   0   0   1   0   0   0     % phi
             0   0   0   0   1   0   0     % psi    
             0   0   0   0   0   1   0     % Alierons         % Controls
             0   0   0   0   0   0   1     % Rudder
             0   0   0   0   0   0   0     % Alierons Command % Commands
             0   0   0   0   0   0   0     % Rudder Command
             0   0   0   0   0  -10  0     % Alieron Rate     % Rates
             0   0   0   0   0   0  -10];  % Rudder Rate
    D_new = zeros(11,2);
    D_new(8,1) = 1;
    D_new(9,2) = 1;
    D_new(10,1) = 10;
    D_new(11,2) = 10;

end