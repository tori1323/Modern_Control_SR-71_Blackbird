
% =============== Matrices Describing SR-71 Dynamics ===============
% Longitudinal Dynamics
% x = [u; alpha; q; theta]
% u = [delta_e; delta_T]
A_long = [0.00109     -29.94  -0.00056    -31.29;
         -0.00000433  -0.144      1     -0.00000151;
          0.000129    -0.0909  -0.104    -0.00000246;
              0          0        1           0];
B_long = [-1.356      5.365;
         -0.00731  -0.000234;
          -1.346        0;
             0          0];
% Laternal Dynamics
% x = [beta; p; r; phi; psi]
% u = [delta_a; delta_r]
A_lat = [-0.0346    0.114   -0.993    0.0101    0.0000174;
         -1.393    -0.261   0.0221   0.0000426  0.0000156;
          0.804    0.00225  -0.0183 -0.0000245      0;
        -0.000149     1      0.113   0.0000171  0.0000503;
        0.0000399     0        1         0      0.0000625];
B_lat = [0     0.00639;
       2.705    0.348;
      -0.0446  -0.938;
         0        0;
         0        0];
C = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 0;
     0 0 0 1 0;
     0 0 0 0 1];
D = zeros(5,2);

% ====================== Properties of SR-71 ======================
M = 3.2; % Mach
H = 76500; % [ft] Cruise Altitude
alpha = 6.56; % [deg] Angle of Attack
q_bar = 495.59; % [psf] Dynamic Pressure
delta_e = -1.81; % [deg] Control Elevator
delta_T = 71; % [deg] Throttle Lever Angle

% ======================== Save Variables  ========================
file_name = 'SR_71_Dynamics.mat';
save(file_name,'A_long','A_lat','B_long','B_lat','C','D')