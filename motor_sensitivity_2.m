clc
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
mdlWks.assignin('tq_Mot_Frnt', zeros([1, 46]))

mdlWks.assignin('p_Brk_Re', [time, ones(length(time), 1) * 4])
mdlWks.assignin('p_Brk_Frnt', [time, ones(length(time), 1) * 6])

mdlWks.assignin('initial_speed', limit / 3.6)

out = sim("model.slx");

accx_brk = out.a(out.v > 0 & out.v <= limit) * 9.81;
v_brk = out.v(out.v > 0 & out.v <= limit) / 3.6;

% lateral acceleration
accy_brk = (v_brk .^ 0.2) * 8;

GG_brk = zeros(length(v_brk), 20);

for i = 1:length(v_brk)
    accy = linspace(-accy_brk(i), accy_brk(i), 20);
    GG_brk(i,:) = -sqrt(abs((1 - (abs(accy).^2) / (accy_brk(i)^2))) .* (accx_brk(i)^2));
end

%% Setup
gbx_rat = 10:2:16;
tq_Peak = 20:2:44;
rat_fr = 1:0.5:3;

AutoX = zeros([length(tq_Peak), length(rat_fr), length(gbx_rat)]);
Endurance = zeros([length(tq_Peak), length(rat_fr), length(gbx_rat)]);
Accel = zeros([length(tq_Peak), length(rat_fr), length(gbx_rat)]);

i = 1;

for tq = tq_Peak
    j = 1;  

    for rat = rat_fr
        k = 1;
        [speed_Mot_Re, tq_Mot_Re] = curve(tq, 20000);
        [speed_Mot_Frnt, tq_Mot_Frnt] = curve(tq / rat, 20000);
        
        for gbx = gbx_rat
            if gbx * tq > 420
                Accel(i,j,k) = NaN;
                AutoX(i,j,k) = NaN;
                Endurance(i,j,k) = NaN;

                disp((100 * ((i-1) * length(rat_fr) * length(gbx_rat) + (j-1) * length(gbx_rat) + k)) / (length(rat_fr) * length(tq_Peak) * length(gbx_rat)) + "%")

                k = k + 1;
                continue
            end

            %% GG accelerating
            limit = 110;

            simtime = 10;
            mdlWks.assignin('simtime', simtime)
            
            time = (0:0.1:simtime)';
    
            mdlWks.assignin('tq_Mot_Re', tq_Mot_Re * 2)
            mdlWks.assignin('tq_Mot_Frnt', tq_Mot_Frnt)
            mdlWks.assignin('speed_Mot_Re', speed_Mot_Re)
            
            mdlWks.assignin('p_Brk_Re', [time, zeros(length(time), 1)])
            mdlWks.assignin('p_Brk_Frnt', [time, zeros(length(time), 1)])
            
            mdlWks.assignin('initial_speed', 0.2)
            
            out = sim("model.slx");
            
            accx_acc = out.a(out.v<=limit) * 9.81;
            v_acc = out.v(out.v<=limit) / 3.6;

            accy_acc = (v_acc .^ 0.2) * 8;
            
            % elliptical lateral acceleration   
            GG_acc = zeros(length(v_acc), 20);  

            for l = 1:length(v_acc)
                accy = linspace(-accy_acc(l), accy_acc(l), 20);
                GG_acc(l,:) = sqrt(abs((1 - (abs(accy).^2) / (accy_acc(l)^2))) .* (accx_acc(l)^2));
            end
            
            % convert to SI for easier calculation
            limit = limit / 3.6;
    
            %% Simulating
            Accel(i,j,k) = accel(GG_acc, v_acc, limit);
            AutoX(i,j,k) = autox(GG_acc, GG_brk, accy_acc, v_acc, v_brk, limit);
            Endurance(i,j,k) = endurance(GG_acc * 0.65, GG_brk, accy_acc, v_acc, v_brk, limit);
    
            disp((100 * ((i-1) * length(rat_fr) * length(gbx_rat) + (j-1) * length(gbx_rat) + k)) / (length(rat_fr) * length(tq_Peak) * length(gbx_rat)) + "%")

            k = k + 1;
        end

        j = j + 1;
    end

    i = i + 1;
end

%% Plots
figure
surf(rat_fr, tq_Peak, Accel(:,:,1), 'FaceColor', '#0072BD')
hold on
surf(rat_fr, tq_Peak, Accel(:,:,2), 'FaceColor', '#D95319')
surf(rat_fr, tq_Peak, Accel(:,:,3), 'FaceColor', '#EDB120')
surf(rat_fr, tq_Peak, Accel(:,:,4), 'FaceColor', '#7E2F8E')
ylabel("Peak Torque rear [Nm]")
xlabel("Torque ratio front to back [ ]")
zlabel("Laptime [s]")
legend('Gearbox = 1:10', 'Gearbox = 1:12', 'Gearbox = 1:14', 'Gearbox = 1:16')
title("Acceleration")

figure
surf(rat_fr, tq_Peak, AutoX(:,:,1), 'FaceColor', '#0072BD')
hold on
surf(rat_fr, tq_Peak, AutoX(:,:,2), 'FaceColor', '#D95319')
surf(rat_fr, tq_Peak, AutoX(:,:,3), 'FaceColor', '#EDB120')
surf(rat_fr, tq_Peak, AutoX(:,:,4), 'FaceColor', '#7E2F8E')
ylabel("Peak Torque rear [Nm]")
xlabel("Torque ratio front to back [ ]")
zlabel("Laptime [s]")
legend('Gearbox = 1:10', 'Gearbox = 1:12', 'Gearbox = 1:14', 'Gearbox = 1:16')
title("AutoX")

figure
surf(rat_fr, tq_Peak, Endurance(:,:,1), 'FaceColor', '#0072BD')
hold on
surf(rat_fr, tq_Peak, Endurance(:,:,2), 'FaceColor', '#D95319')
surf(rat_fr, tq_Peak, Endurance(:,:,3), 'FaceColor', '#EDB120')
surf(rat_fr, tq_Peak, Endurance(:,:,4), 'FaceColor', '#7E2F8E')
ylabel("Peak Torque rear [Nm]")
xlabel("Torque ratio front to back [ ]")
zlabel("Laptime [s]")
legend('Gearbox = 1:10', 'Gearbox = 1:12', 'Gearbox = 1:14', 'Gearbox = 1:16')
title("Endurance (one lap)")

%% Functions
function [speed_Mot, tq_Mot] = curve(tq_Peak, speed_Corner)

speed_Mot = 0:500:22500;

tq_Mot = NaN(size(speed_Mot));
tq_Mot(end) = tq_Peak / 2;
tq_Mot(1:speed_Corner / 500) = tq_Peak;
tq_Mot = fillmissing(tq_Mot, 'linear');

end
