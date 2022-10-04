%% Tires
rad_Tire_Dyn = 0.2032;      % Dynamic Tire Radius       [m]
coeff_Roll = 0.005;         % traction coefficient tire [ ]
I_Whl = 2;                  % Inertia Wheel             [kg*m^2]

%% Drivetrain
rat_Gbx = 13.8;             % effective gearbox ratio   [ ]
I_Mot = 1;                  % Inertia Motor             [kg*m^2]

%% Brakes
diam_Pist_Re = 2.54;        % piston diameter rear      [cm]
diam_Pist_Frnt = 2.54;      % piston diameter front     [cm]
nu_Pist_Re = 2;             % number of pistons rear    [ ]
nu_Pist_Frnt = 4;           % number of pistons front   [ ]
rad_BrkDisc_Re = 0.1725;    % Radius Brakedisc rear     [m]
rad_BrkDisc_Frnt = 0.1725;  % Radius Brakedisc front    [m]
mu_Brk = 0.56;              % traction coefficient brake[ ]

%% Chassis
m_Veh = 290;                % Vehicle Mass with Drive   [kg]
dst_Track_Re = 0.5;         % Half Track Widht Rear     [m]
dst_Track_Frnt = 0.5;       % Half Track Width Front    [m]
dst_WhlBase_Re = 0.821;     % Distance Rear Axle - Cog  [m]
dst_WhlBase_Frnt = 0.714;   % Distance Front Axle - Cog [m]
coeff_WhlLoad_Re = 0.534;   % Amount Mass on rear Axle  [ ]
coeff_WhlLoad_Frnt = 0.466; % Amount Mass on front Axle [ ]
A_Veh = 1.05;               % Frontal Area              [m^2]
height_Cog = 0.25;          % Height Cog                [m]
I_Veh = 80;                 % Inertia vehicle body      [kg*m^2]

%% Aero
coeff_Veh_Lift = 5.2;       % Coefficient of Lift       [ ]
coeff_Veh_Drag = 1.35;      % Coefficient of Drag       [ ]
coeff_LiftLoad_Re = 0.53;   % Downforce on Rear Axle    [ ]
coeff_LiftLoad_Frnt = 0.53; % Downforce on Front Axle   [ ]
coeff_DragLoad_Re = 0.1;    % Coefficient Wheelload Drag[ ]
coeff_DragLoad_Frnt = 0.1;  % Coefficient Wheelload Drag[ ]

%% Environment
dens_Air = 1.2041;          % Air Density               [kg/m^3]
