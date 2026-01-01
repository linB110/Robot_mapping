function laser = read_robotlaser(filename)

fid = fopen(filename, 'r');
if fid == -1
    error("Cannot open file %s", filename);
end

laser = {};
while true
    ln = fgetl(fid);
    if ~ischar(ln)
        break;
    end

    tokens = strsplit(strtrim(ln));
    if ~strcmp(tokens{1}, "ROBOTLASER1")
        continue;
    end

    num_tokens = str2double(tokens);

    currentReading = struct( ...
        "start_angle", 0, ...
        "angular_resolution", 0, ...
        "maximum_range", 0, ...
        "ranges", [], ...
        "pose", zeros(3,1), ...
        "laser_offset", zeros(3,1), ...
        "timestamp", 0 ...
    );

    tk = 3;
    currentReading.start_angle = num_tokens(tk); tk = tk + 1;
    tk = tk + 1; % skip FOV
    currentReading.angular_resolution = num_tokens(tk); tk = tk + 1;
    currentReading.maximum_range = num_tokens(tk); tk = tk + 1;

    tk = tk + 2; % skip accuracy, remission_mode

    num_readings = int32(num_tokens(tk)); tk = tk + 1;
    currentReading.ranges = num_tokens(tk:tk+num_readings-1);
    tk = tk + num_readings;

    num_remissions = int32(num_tokens(tk)); tk = tk + 1;
    tk = tk + num_remissions; % skip remission values

    laser_pose = num_tokens(tk:tk+2);
    tk = tk + 3;

    currentReading.pose = num_tokens(tk:tk+2);
    tk = tk + 3;

    currentReading.laser_offset = ...
        t2v(inv(v2t(currentReading.pose)) * v2t(laser_pose));

    tk = tk + 5; % skip tv, rv, forward, side, turn

    currentReading.timestamp = num_tokens(tk);

    laser{end+1} = currentReading;
end

fclose(fid);
laser = cell2mat(laser);

end
