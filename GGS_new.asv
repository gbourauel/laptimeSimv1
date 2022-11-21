clear
close all

open('model.slx')
malgucka
mdlWks = get_param('model', 'ModelWorkspace');

%% longitudinal bullshit
% GG breaking        
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

accx_brk = out.a(out.v > 0.2 & out.v <= limit);
vel_brk = out.v(out.v > 0.2 & out.v <= limit);

% GG accelerating
[speed_Mot_Re, tq_Mot_Re] = curve(29.1, 13000);

limit = 110;

simtime = 10;
mdlWks.assignin('simtime', simtime)

time = (0:0.1:simtime)';

mdlWks.assignin('tq_Mot_Re', tq_Mot_Re * 2)
mdlWks.assignin('tq_Mot_Frnt', tq_Mot_Re * 2)
mdlWks.assignin('speed_Mot_Re', speed_Mot_Re)

mdlWks.assignin('p_Brk_Re', [time, zeros(length(time), 1)])
mdlWks.assignin('p_Brk_Frnt', [time, zeros(length(time), 1)])

mdlWks.assignin('initial_speed', 0.2)

out = sim("model.slx");

accx_acc = out.a(out.v<=limit);
vel_acc = out.v(out.v<=limit);

%% SI FOR FUCKS SAKE
accx_acc = accx_acc * 9.81;
accx_brk = accx_brk * 9.81;

vel_acc = vel_acc / 3.6;
vel_brk = vel_brk / 3.6;

%% lateral bullshit
accy_acc = (vel_acc .^ 0.2) * 8;
accy_brk = (vel_brk .^ 0.2) * 8;

%% GG Ellipsis
GG_acc = zeros(length(vel_acc), 20);

figure
hold on
grid on

for i = 1:length(vel_acc)
    accy = linspace(-accy_acc(i), accy_acc(i), 20);
    GG_acc(i,:) = sqrt(abs((1 - (abs(accy).^2) / (accy_acc(i)^2))) .* (accx_acc(i)^2));

    temp = ones(size(accy)) * vel_acc(i);
    plot3(temp * 3.6, accy / 9.81, GG_acc(i,:) / 9.81)
end

GG_brk = zeros(length(vel_brk), 20);

for i = 1:length(vel_brk)
    accy = linspace(-accy_brk(i), accy_brk(i), 20);
    GG_brk(i,:) = -sqrt(abs((1 - (abs(accy).^2) / (accy_brk(i)^2))) .* (accx_brk(i)^2));

    temp = ones(size(accy)) * vel_brk(i);
    plot3(temp * 3.6, accy / 9.81, GG_brk(i,:) / 9.81)
end

clear temp accy

xlabel('Velocity [km/h]')
ylabel('Lateral acceleration [g]')
zlabel('Longitudinal acceleration [g]')
hold off

%% Indexing
radius = 9;

for i = fliplr(1:length(vel_acc))
    accy = accy_acc(i);

    vel = sqrt(accy * radius);

    if vel <= vel_acc(i)
        break
    end
end

disp(vel * 3.6)

%% Functions
function [speed_Mot, tq_Mot] = curve(tq_Peak, speed_Corner)

speed_Mot = 0:500:22500;

tq_Mot = NaN(size(speed_Mot));
tq_Mot(end) = tq_Peak / 4;
tq_Mot(1:speed_Corner / 500) = tq_Peak;
tq_Mot = fillmissing(tq_Mot, 'linear');

end
