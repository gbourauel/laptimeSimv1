function laptime = autox(GG_acc, GG_brk, accy, accy_max, v_acc, v_brk, limit)

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
