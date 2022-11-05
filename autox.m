function laptime = autox()

%% GGS
% accelerating
malgucka
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

% breaking
simtime = 5;
mdlWks.assignin('simtime', simtime)

time = (0:0.1:simtime)';

mdlWks.assignin('tq_Mot_Re', [0 0 0 0 0 0 0] * 2)

mdlWks.assignin('p_Brk_Re', [time, ones(length(time), 1) * 4])
mdlWks.assignin('p_Brk_Frnt', [time, ones(length(time), 1) * 6])

mdlWks.assignin('initial_speed', max(v_acc) / 3.6)

out = sim("model.slx");

accx_brk = out.a(out.v > 0 & out.v <= max(v_acc));
v_brk = out.v(out.v > 0 & out.v <= max(v_acc));

% elliptical lateral acceleration
accy_max = 1.6;
accy_min = -1.6;

accy = accy_min:0.1:accy_max;

GG_acc = sqrt(abs((1-(abs(accy).^2)/(abs(accy_max)^2))).*(accx_acc.^2));
GG_brk = -sqrt(abs((1-(abs(accy).^2)/(abs(accy_max)^2))).*(accx_brk.^2));

% convert to SI for easier calculation
GG_acc = GG_acc .* 9.81;
GG_brk = GG_brk .* 9.81;
accy = accy .* 9.81;
accy_max = accy_max * 9.81;

v_acc = v_acc ./ 3.6;
v_brk = v_brk ./ 3.6;
limit = limit / 3.6;

%% Track
% loading track data
load('FSG22.txt')
load('enduranceFSG.txt')

dst_Rd_Track = FSG22(:,4);

v_GSS = enduranceFSG(:,2);

% dividing in segments with corresponding radius
section_length = 0.2;

dst_Rd_Track = movavgFilt(dst_Rd_Track', 71, "Center")';
dst = cumtrapz(v_GSS ./ 3.6) .* 0.001;

track = zeros(ceil(dst(end) / section_length), 1);

for i = 1:length(track)
    dst = dst - section_length;
    track(i) = mean(dst_Rd_Track((0 < dst) & (dst < section_length)));
end

track(abs(track) > 60) = 0;
track(abs(track) < 7) = 7;

% ignore negative radii, because GGS is symmetric
track = abs(track);

% finding every apex
track2 = zeros(length(track), 1);

for i = 2:length(track)-1
    if track(i - 1) > track(i) && track(i) < track(i + 1) && track(i) ~= 0
        track2(i) = track(i);
    end
end

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
for section = track2'
    % straights
    if section == 0
        % indexing
        [~,index] = min(abs(v_acc - v_exit(i)));
        a_exit(i) = GG_acc(index, accy == 0);

    % curves
    else
        % better calculation for accy_max needed when full GG-Diagram is
        % working!!!!!
        v_entry = sqrt(accy_max * section);

        % breaking in previous sectors to match entry speed
        k = i;

        while v_exit(i) > v_entry
            for j = k:i
                [~,index] = min(abs(v_brk - v_exit(j)));
                a_exit(j) = GG_brk(index, accy == 0);
        
                laptime(j+1) = section_length / (v_exit(j));
            
                v_exit(j+1) = a_exit(j) * laptime(j+1) + v_exit(j);
            end

            k = k - 1;
        end
    end

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
