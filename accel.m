function laptime = accel()

%% GGS
% accelerating
limit = 110;

open('model.slx')
mdlWks = get_param('model', 'ModelWorkspace');

simtime = 4.5;
mdlWks.assignin('simtime', simtime)

time = (0:0.1:simtime)';

mdlWks.assignin('tq_Mot_Re', [29.1 29.1 29.1 26 18 15 0] * 2)

mdlWks.assignin('p_Brk_Re', [time, zeros(length(time), 1)])
mdlWks.assignin('p_Brk_Frnt', [time, zeros(length(time), 1)])

mdlWks.assignin('initial_speed', 0.2)

out = sim("model.slx");

accx_acc = out.a(out.v<=limit);
v_acc = out.v(out.v<=limit);

% elliptical lateral acceleration
accy_max = 1.6;
accy_min = -1.6;

accy = accy_min:0.1:accy_max;

GG_acc = sqrt(abs((1-(abs(accy).^2)/(abs(accy_max)^2))).*(accx_acc.^2));

% convert to SI for easier calculation
GG_acc = GG_acc .* 9.81;
accy = accy .* 9.81;
accy_max = accy_max * 9.81;

v_acc = v_acc ./ 3.6;
limit = limit / 3.6;

%% Track
section_length = 0.5;
track2 = 1:section_length:75;

%% Laptime
% setup
v_exit = zeros(length(track2) + 1, 1);
laptime = zeros(length(track2) + 1, 1);
a_exit = zeros(length(track2) - 1, 1);

i = 1;

% launch
[~,index] = min(abs(v_acc - v_exit(i)));
a_exit(i) = GG_acc(index, accy == 0);

a_exit(i) = GG_acc(index + 1, accy == 0);
laptime(i+1) = sqrt(2 / (a_exit(i)));
v_exit(i) = a_exit(i) * laptime(i+1) + v_exit(i);

% forward calculation
for section = track2
    % indexing
    [~,index] = min(abs(v_acc - v_exit(i)));
    a_exit(i) = GG_acc(index, accy == 0);

    % "driving"
    laptime(i+1) = section_length / (v_exit(i));
    v_exit(i+1) = a_exit(i) * laptime(i+1) + v_exit(i);

    % motor speed limit
    if v_exit(i+1) >= limit
        v_exit(i+1) = limit;
    end
    
    i = i + 1;
end

laptime = sum(laptime);

end
