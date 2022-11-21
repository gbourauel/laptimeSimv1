function laptime = accel(GG_acc, v_acc, limit)

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
a_exit(i) = max(GG_acc(index, :));

laptime(i+1) = sqrt(2 / (a_exit(i)));
v_exit(i) = a_exit(i) * laptime(i+1) + v_exit(i);

% forward calculation
for section = track2
    % indexing
    [~,index] = min(abs(v_acc - v_exit(i)));
    a_exit(i) = max(GG_acc(index, :));

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
