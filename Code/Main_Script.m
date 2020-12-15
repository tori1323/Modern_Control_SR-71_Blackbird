% Victoria Nagorski - Aero 625 - Modern Control Project
% Version 2.0 - 11/25/20
% Modern Control Design for SR-71 (Blackbird)

%% ======================== Start Code =========================
% Load Variables
clear;close all;clc;
file_name = 'SR_71_Dynamics.mat';
load(file_name)
clear A_long B_long file_name; % Clear Up Memory Space
H = [0 0 0 0 1 0 0]; % Define the H Matrix

%% ====================== Add Actuators =========================
[A_new,B_new,C_new,D_new] = add_actuators_sys(A_lat,B_lat,C,D)
sys = ss(A_new,B_new,C_new,D_new); % Create the new system
clear A_lat B_lat C D

%% ================== Open System Properties =====================
% Eigenvalues (D) & Eigenvectors (V)
[V,D] = eig(A_new)

% Natural Frequencies & Damping Ratios
damp(sys)

% Modal Analysis
[real_evm,Am_real] = cdf2rdf(V,D);
Modal_Matrix = real_evm
Am = inv(Modal_Matrix) * A_new * Modal_Matrix
Bm = inv(Modal_Matrix) * B_new
Cm = C_new * Modal_Matrix
Z = inv(Modal_Matrix)

% Time Constant
T = 0.6;
pacing.T = T;
pacing.h = .01;

% Verification of Controllability 
Cont = ctrb(A_new,B_new)
if rank(Cont) == 7
    Control = "True"
end

% Verification of Observability 
Obs = obsv(A_new,C_new)

if rank(Obs) == 7
    Observable = "True"
end

% Bode Plot
G = tf(sys);
figure
sigma(G)
grid
%% =========================== SDR ===========================
% Selected Elements
close all;
ss2.Q = [50   0     0  0    0     0  0; 
         0  1e-100  0  0    0     0  0;
         0    0     5  0    0     0  0;
         0    0     0  1    0     0  0;
         0    0     0  0  1e-100  0  0;
         0    0     0  0    0     1  0;
         0    0     0  0    0     0  1];
ss2.R = [8 0;
         0 65];
Q = ss2.Q
R = ss2.R
     
% Set-Up
ss2.A = sys.A; 
ss2.B = sys.B;
ss2.C = sys.C;
ss2.D = sys.D;
t_final = 20; % Final Time
table.x = [10*(pi/180);0;0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline

% Simulate System
[gains,table] = SDR(ss2,pacing,table,t_final);
Title = "SR-71 Lateral Dynamics"';
graphing(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% Solve for Closed Loop Matrix
A_cl = A_new - (B_new * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys2 = ss(A_cl,B_new,C_new,[0]);
damp(sys2)

% Bode Plot
G = tf(sys2);
figure
sigma(G)
grid

%% ========================= PI - SDR ==========================
clear table;close all;
% Selected Elements
ss3.Q = [1  0  0  0   0  0  0  0; 
         0  1  0  0   0  0  0  0;
         0  0  1  0   0  0  0  0;
         0  0  0  10  0  0  0  0; % Might change this one
         0  0  0  0  10  0  0  0; % less likely to change but maybe
         0  0  0  0   0  1  0  0;
         0  0  0  0   0  0  1  0;
         0  0  0  0   0  0  0  10]; 
ss3.R = [80 0;
         0 100];
Q = ss3.Q
R = ss3.R

% Set-Up
ss3.A = sys.A;
ss3.B = sys.B;
ss3.C = sys.C;
ss3.D = sys.D;
ss3.H = [0 0 1 0 0 0 0]; 
t_final = 20; % Final Time
table.x = [0;0;5*(pi/180);0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline
Disturbance = [0;0.5;0;0;0;0;0;0];


% Simulate System
[gains,table,sys_new] = PI_SDR(ss3,pacing,table,t_final,Disturbance);
Title = "SR-71 Lateral Dynamics"';
graphing(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% New State Equations
A_new_P = sys_new.A
B_new_P = sys_new.B

% Solve for Closed Loop Matrix
A_cl = sys_new.A - (sys_new.B * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys3 = ss(A_cl,sys_new.B,sys_new.C,sys_new.D);
damp(sys3)

% Bode Plot
G = tf(sys3);
figure
sigma(G)
grid

%% ======================== NZSP - Step ========================
clear table;clc;close all;
% Selected Elements
ss4.Q = [1e-100    0     0    0       0   0   0; 
         0       1e-100  0    0       0   0   0;
         0         0     1    0       0   0   0;
         0         0     0  1e-100    0   0   0;
         0         0     0    0      30   0   0;
         0         0     0    0       0   3   0;
         0         0     0    0       0   0   5]; 
ss4.R = [1   0;
         0   40];
Q = ss4.Q
R = ss4.R

% Set-Up
ss4.A = sys.A;
ss4.B = sys.B;
ss4.C = sys.C;
ss4.D = sys.D;
t_final = 20; % Final Time
table.x = [0;0;0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline
ym = 5 * (pi/180); % Track 45 degrees

% Simulate System
[gains,table] = NZSP(ss4,pacing,table,t_final,ym,H);
Title = "SR-71 Lateral Dynamics"';
graphing_NZSP(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% Steady State Values
x_star = table.pi12 * ym
u_star = table.pi22 * ym

% Solve for Closed Loop Matrix
A_cl = A_new - (B_new * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys4 = ss(A_cl,B_new,C_new,[0]);
damp(sys4)

%% ====================== NZSP - Sinusoid =======================
clear table;clc;close all;
% Selected Elements
ss4.Q = [1e-100  0      0  0   0  0       0; 
         0      1e-100  0  0   0  0       0;
         0       0      1  0   0  0       0;
         0       0      0  10  0  0       0;
         0       0      0  0  50  0       0;
         0       0      0  0   0  1e-100  0;
         0       0      0  0   0  0       1]; 
ss4.R = [7 0;
         0 25];
Q = ss4.Q
R = ss4.R

% Set-Up
ss4.A = sys.A;
ss4.B = sys.B;
ss4.C = sys.C; % H matrix
ss4.D = sys.D;
t_final = 20; % Final Time
table.x = [0;0;0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline
frequency = (2*pi)/(16);
ym_sin = ym .* sin(frequency .* table.t); % Track sinusoid

% Simulate System
[gains,table] = NZSP_sin(ss4,pacing,table,t_final,ym_sin,H);
Title = "SR-71 Lateral Dynamics"';
table.track = ym_sin;
graphing_NZSP(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% Solve for Closed Loop Matrix
A_cl = A_new - (B_new * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys4 = ss(A_cl,B_new,C_new,[0]);
damp(sys4)

%% ====================== PI - NZSP - Step =======================
clear table;clc;close all;
% Selected Elements 
ss4.Q = [1  0   0   0   0   0   0  0; 
         0  1   0   0   0   0   0  0;
         0  0   1   0   0   0   0  0;
         0  0   0   1   0   0   0  0; % Come back to this one
         0  0   0   0   1   0   0  0; 
         0  0   0   0   0   1   0  0; 
         0  0   0   0   0   0   1  0;
         0  0   0   0   0   0   0  1]; % Come back to 
ss4.R = [9 0; 
         0 70];
Q = ss4.Q
R = ss4.R

% Set-Up
ss4.A = sys.A;
ss4.B = sys.B;
ss4.C = sys.C;
ss4.D = sys.D;
ss4.H = H;
t_final = 20; % Final Time
table.x = [0;0;0;0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline
ym = 5 * (pi/180); % Track 45 degrees
Disturbance = [0;0.5;0;0;0;0;0;0];
pacing.T = .05;

% Simulate System
[gains,table,sys_new] = PI_NZSP(ss4,pacing,table,t_final,ym,Disturbance);
Title = "SR-71 Lateral Dynamics"';
graphing_NZSP(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% Steady State Values
x_star = table.pi12 * ym
u_star = table.pi22 * ym

% New State Equations
A_new = sys_new.A
B_new = sys_new.B

% Solve for Closed Loop Matrix
A_cl = sys_new.A - (sys_new.B * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys4 = ss(A_cl,sys_new.B,sys_new.C,sys_new.D);
damp(sys4)

%% ==================== PI - NZSP - Sinusoid ======================
clear table;clc;close all;
% Selected Elements
ss4.Q = [1  0   0  0  0   0  0 0; 
         0  50  0  0  0   0  0 0;
         0  0  10  0  0   0  0 0;
         0  0   0  1  0   0  0 0;
         0  0   0  0  49  0  0 0;
         0  0   0  0  0   3  0 0;
         0  0   0  0  0   0  7 0;
         0  0   0  0  0   0  0 1]; 
ss4.R = [1.7 0;
         0 30];
Q = ss4.Q
R = ss4.R

% Set-Up
ss4.A = sys.A;
ss4.B = sys.B;
ss4.C = sys.C;
ss4.D = sys.D;
ss4.H = H;
t_final = 20; % Final Time
table.x = [0;0;0;0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline
frequency = (2*pi)/(16);
ym_sin = ym .* sin(frequency .* table.t); % Track sinusoid
Disturbance = [0;0.5;0;0;0;0;0;0];

% Simulate System
[gains,table,sys_new] = PI_NZSP_sin(ss4,pacing,table,t_final,ym_sin,Disturbance);
Title = "SR-71 Lateral Dynamics"';
table.track = ym_sin;
graphing_NZSP(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% New State Equations
A_new = sys_new.A
B_new = sys_new.B

% Solve for Closed Loop Matrix
A_cl = sys_new.A - (sys_new.B * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys4 = ss(A_cl,sys_new.B,sys_new.C,sys_new.D);
damp(sys4)

%% ================== PIF - NZSP - CRW - Step =====================
clear table;clc;close all;
% Selected Elements
ss4.Q = [1  0  0   0  0    0  0   0  0  0; 
         0  1  0   0  0    0  0   0  0  0;
         0  0  15  0  0    0  0   0  0  0;
         0  0  0   1  0    0  0   0  0  0;
         0  0  0   0  100  0  0   0  0  0; %end of states
         0  0  0   0  0   10  0   0  0  0; 
         0  0  0   0  0    0  10  0  0  0;
         0  0  0   0  0    0  0  20  0  0;
         0  0  0   0  0    0  0   0  1  0;
         0  0  0   0  0    0  0   0  0  1];  
ss4.R = [1  0;
         0 3.2];
Q = ss4.Q
R = ss4.R

% Set-Up
ss4.A = sys.A;
ss4.B = sys.B;
ss4.C = sys.C;
ss4.D = sys.D;
ss4.H = H;
t_final = 20; % Final Time
table.x = [0;0;0;0;0;0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline
ym = 5 * (pi/180); % Track 5 degrees
Disturbance = [0;0.5;0;0;0;0;0;0;0;0];

% Simulate System
[gains,table,sys_new] = PIF_NZSP_CRW(ss4,pacing,table,t_final,ym,Disturbance);
Title = "SR-71 Lateral Dynamics"';
graphing_PIF_NZSP_CRW(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% Steady State Values
x_star = table.pi12 * ym
u_star = table.pi22 * ym

% New State Equations
A_new = sys_new.A
B_new = sys_new.B

% Solve for Closed Loop Matrix
A_cl = sys_new.A - (sys_new.B * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys4 = ss(A_cl,sys_new.B,sys_new.C,sys_new.D);
damp(sys4)

%% ================ PIF - NZSP - CRW - Sinusoid ===================
clear table;clc;close all;
% Selected Elements
ss4.Q = [50  0  0   0   0  0  0   0  0  0; 
         0  50  0   0   0  0  0   0  0  0;
         0   0  1   0   0  0  0   0  0  0;
         0   0  0  100  0  0  0   0  0  0;
         0   0  0   0  50  0  0   0  0  0; %end of states
         0   0  0   0   0  1  0   0  0  0; 
         0   0  0   0   0  0  22  0  0  0;
         0   0  0   0   0  0  0   1  0  0;
         0   0  0   0   0  0  0   0  1  0;
         0   0  0   0   0  0  0   0  0  1];  
ss4.R = [1 0;
         0 1];
Q = ss4.Q
R = ss4.R

% Set-Up
ss4.A = sys.A;
ss4.B = sys.B;
ss4.C = sys.C;
ss4.D = sys.D;
ss4.H = H;
t_final = 20; % Final Time
table.x = [0;0;0;0;0;0;0;0;0;0]; % Initialize position
table.t = 0:.01:t_final; % Timeline
ym = 5 * (pi/180); % Track 5 degrees
frequency = (2*pi)/(16);
ym_sin = ym .* sin(frequency .* table.t); % Track sinusoid
Disturbance = [0;0.5;0;0;0;0;0;0;0;0];
pacing.T = .05;

% Simulate System
[gains,table,sys_new] = PIF_NZSP_CRW_sin(ss4,pacing,table,t_final,ym_sin,Disturbance);
Title = "SR-71 Lateral Dynamics"';
table.track = ym_sin;
graphing_PIF_NZSP_CRW(table,Title)
Q_hat = gains.Q_hat
R_hat1 = gains.R_hat
M1 = gains.M
K = gains.K

% Steady State Values
x_star = table.pi12 * ym
u_star = table.pi22 * ym

% New State Equations
A_new = sys_new.A
B_new = sys_new.B

% Solve for Closed Loop Matrix
A_cl = sys_new.A - (sys_new.B * K)
% Eigenvalues (D) & Eigenvectors (V)
[V_cl,D_cl] = eig(A_cl)
% Natural Frequencies & Damping Ratios
sys4 = ss(A_cl,sys_new.B,sys_new.C,sys_new.D);
damp(sys4)

%% ==================== Error Graph for Step ======================
close all;
% NOTE: Run this section after you run one of the step sections.  If you
% run right after a sinusiodal section, then the graphs will look
% incorrect.

hold on
title('Error Function')
ylim([0,6])
xlabel('Time')
ylabel('[Radians]')
plot(table.t(1,:),(180/pi)*table.track(1,:))
hold off