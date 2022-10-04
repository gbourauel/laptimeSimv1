% vehicle parameters
mVeh = 282;         % mass vehicle with driver      [kg]
dstRdTir = 0.2032;  % radius tire                   [m]
gbx = 13.8 * 0.95;  % ratio gearbox effective       [ ]
coeffRoll = 0.025;  % coefficient rollresistance    [ ]
AVeh = 1.05;        % frontal area                  [m^2]
coeffAir = 1.35;    % coefficient airresistance     [ ]
coeffLift = 5.2;    % coefficient lift              [ ]
mTir = 5.3;         % mass tire                     [kg]
IWhl = 3;           % Inertia tire                  [kg*m^2]
IMot = 2;           % inertia motor                 [kg*m^2]

% environment parameters (no need to change)
coeffGrvy = 9.81;   % coefficient gravity           [N/kg]
densAir = 1.225;    % air density                   [kg/m^3]
