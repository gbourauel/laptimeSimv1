clear
close all

open('model.slx')
malgucka
mdlWks = get_param('model', 'ModelWorkspace');

%% GG breaking        
limit = 110;

simtime = 5;
mdlWks.assignin('simtime', simtime)

time = (0:0.1:simtime)';

mdlWks.assignin('tq_Mot_Re', zeros([1, 46]))

mdlWks.assignin('p_Brk_Re', [time, ones(length(time), 1) * 4])
mdlWks.assignin('p_Brk_Frnt', [time, ones(length(time), 1) * 6])

mdlWks.assignin('initial_speed', limit / 3.6)

out = sim("model.slx");

accx_brk = out.a(out.v > 0 & out.v <= limit);
v_brk = out.v(out.v > 0 & out.v <= limit);

% lateral acceleration
accy_max = 1.6;
accy_min = -1.6;

accy = accy_min:0.1:accy_max;

GG_brk = -sqrt(abs((1-(abs(accy).^2)/(abs(accy_max)^2))).*(accx_brk.^2));

% convert to SI for easier calculation
GG_brk = GG_brk * 9.81;

accy = accy * 9.81;
accy_max = accy_max * 9.81;

v_brk = v_brk / 3.6;
limit = limit / 3.6;

%% Setup
gbx_rat = 8:4:16;
tq_Peak = 18:4:38;
speed_Corner = 10000:2000:22000;

AutoX = zeros([length(tq_Peak), length(speed_Corner), length(gbx_rat)]);
Endurance = zeros([length(tq_Peak), length(speed_Corner), length(gbx_rat)]);
Accel = zeros([length(tq_Peak), length(speed_Corner), length(gbx_rat)]);

i = 1;

for tq = tq_Peak
    j = 1;  

    for speed = speed_Corner
        k = 1;
        [speed_Mot_Re, tq_Mot_Re] = curve(tq, speed);
        
        for gbx = gbx_rat
            %% GG accelerating
            simtime = 4.5;
            mdlWks.assignin('simtime', simtime)
            
            time = (0:0.1:simtime)';
    
            mdlWks.assignin('tq_Mot_Re', tq_Mot_Re * 2)
            mdlWks.assignin('speed_Mot_Re', speed_Mot_Re * 2)
            
            mdlWks.assignin('p_Brk_Re', [time, zeros(length(time), 1)])
            mdlWks.assignin('p_Brk_Frnt', [time, zeros(length(time), 1)])
            
            mdlWks.assignin('initial_speed', 0.2)
            
            out = sim("model.slx");
            
            accx_acc = out.a(out.v<=limit);
            v_acc = out.v(out.v<=limit);
            
            % elliptical lateral acceleration     
            GG_acc = sqrt(abs((1-(abs(accy).^2)/(abs(accy_max)^2))).*(accx_acc.^2));
            
            % convert to SI for easier calculation
            GG_acc = GG_acc * 9.81;
            v_acc = v_acc / 3.6;
    
            %% Simulating
            Accel(i,j,k) = accel(GG_acc, accy, v_acc, limit);
            AutoX(i,j,k) = autox(GG_acc, GG_brk, accy, accy_max, v_acc, v_brk, limit);
            Endurance(i,j,k) = endurance(GG_acc, GG_brk, accy, accy_max, v_acc, v_brk, limit);
    
            disp((100 * ((i-1) * length(speed_Corner) * length(gbx_rat) + (j-1) * length(gbx_rat) + k)) / (length(speed_Corner) * length(tq_Peak) * length(gbx_rat)) + "%")

            k = k + 1;
        end

        j = j + 1;
    end

    i = i + 1;
end

%% Plots
figure
surf(speed_Corner, tq_Peak, Accel(:,:,1), 'FaceColor', '#0072BD')
hold on
surf(speed_Corner, tq_Peak, Accel(:,:,2), 'FaceColor', '#D95319')
surf(speed_Corner, tq_Peak, Accel(:,:,3), 'FaceColor', '#EDB120')
% surf(speed_Corner, tq_Peak, Accel(:,:,4), 'FaceColor', '#7E2F8E')
% surf(speed_Corner, tq_Peak, Accel(:,:,5), 'FaceColor', '#77AC30')
ylabel("Peak Torque [Nm]")
xlabel("Corner Speed [rpm]")
zlabel("Laptime [s]")
legend('Gearbox = 1:8', 'Gearbox = 1:12', 'Gearbox = 1:16')
title("Acceleration")

figure
surf(speed_Corner, tq_Peak, AutoX(:,:,1), 'FaceColor', '#0072BD')
hold on
surf(speed_Corner, tq_Peak, AutoX(:,:,2), 'FaceColor', '#D95319')
surf(speed_Corner, tq_Peak, AutoX(:,:,3), 'FaceColor', '#EDB120')
% surf(speed_Corner, tq_Peak, AutoX(:,:,4), 'FaceColor', '#7E2F8E')
% surf(speed_Corner, tq_Peak, AutoX(:,:,5), 'FaceColor', '#77AC30')
ylabel("Peak Torque [Nm]")
xlabel("Corner Speed [rpm]")
zlabel("Laptime [s]")
legend('Gearbox = 1:8', 'Gearbox = 1:12', 'Gearbox = 1:16')
title("AutoX")

figure
surf(speed_Corner, tq_Peak, Endurance(:,:,1), 'FaceColor', '#0072BD')
hold on
surf(speed_Corner, tq_Peak, Endurance(:,:,2), 'FaceColor', '#D95319')
surf(speed_Corner, tq_Peak, Endurance(:,:,3), 'FaceColor', '#EDB120')
% surf(speed_Corner, tq_Peak, Endurance(:,:,4), 'FaceColor', '#7E2F8E')
% surf(speed_Corner, tq_Peak, Endurance(:,:,5), 'FaceColor', '#77AC30')
ylabel("Peak Torque [Nm]")
xlabel("Corner Speed [rpm]")
zlabel("Laptime [s]")
legend('Gearbox = 1:8', 'Gearbox = 1:12', 'Gearbox = 1:16')
title("Endurance (one lap)")

%% Functions
function [speed_Mot, tq_Mot] = curve(tq_Peak, speed_Corner)

speed_Mot = 0:500:22500;

tq_Mot = NaN(size(speed_Mot));
tq_Mot(end) = tq_Peak / 2;
tq_Mot(1:speed_Corner / 500) = tq_Peak;
tq_Mot = fillmissing(tq_Mot, 'spline');

end
