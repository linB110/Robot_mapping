function laser = read_robotlaser(filename)
% READ_ROBOTLASER Read ROBOTLASER1 data from a CARMEN logfile
%
%   laser = read_robotlaser(filename)
%
%   Output:
%     laser : struct array with fields
%       - start_angle
%       - angular_resolution
%       - maximum_range
%       - ranges
%       - pose
%       - laser_offset
%       - timestamp

fid = fopen(filename, 'r');
if fid < 0
    error('Cannot open file: %s', filename);
end

laser = {};  % temporary cell array

while true
    ln = fgetl(fid);
    if ~ischar(ln)
        break;
    end

    tokens = strsplit(strtrim(ln));
    if isempty(tokens)
        continue;
    end

    % Only parse ROBOTLASER1 lines
    if ~strcmp(tokens{1}, 'ROBOTLASER1')
        continue;
    end

    % Convert numeric tokens (skip first string token)
    num_tokens = str2double(tokens(2:end));

    % Replace NaNs from tail (hostname, logger, etc.) with zeros
    num_tokens(isnan(num_tokens)) = 0;

    % Initialize struct
    currentReading = struct( ...
        "start_angle", 0, ...
        "angular_resolution", 0, ...
        "maximum_range", 0, ...
        "ranges", [], ...
        "pose", zeros(3,1), ...
        "laser_offset", zeros(3,1), ...
        "timestamp", 0 ...
    );

    tk = 2;  % index into num_tokens

    % Parse main parameters
    currentReading.start_angle = num_tokens(tk); tk = tk + 1;
    tk = tk + 1; % skip FOV
    currentReading.angular_resolution = num_tokens(tk); tk = tk + 1;
    currentReading.maximum_range = num_tokens(tk); tk = tk + 1;
    tk = tk + 2; % skip accuracy, remission_mode

    % Parse laser ranges
    num_readings = int32(num_tokens(tk)); tk = tk + 1;
    if tk + num_readings - 1 > numel(num_tokens)
        warning('Skipping line: not enough numeric tokens for ranges.');
        continue;
    end
    currentReading.ranges = num_tokens(tk:tk+num_readings-1);
    tk = tk + num_readings;

    % Skip remissions
    num_remissions = int32(num_tokens(tk)); tk = tk + 1;
    tk = tk + num_remissions;

    % Laser pose in world
    laser_pose = num_tokens(tk:tk+2);
    tk = tk + 3;

    % Robot pose in world
    currentReading.pose = num_tokens(tk:tk+2);
    tk = tk + 3;

    % Compute laser offset in robot frame
    currentReading.laser_offset = t2v(inv(v2t(currentReading.pose)) * v2t(laser_pose));

    tk = tk + 5; % skip tv, rv, forward, side, turn

    % Timestamp
    currentReading.timestamp = num_tokens(tk);

    % Append to cell
    laser{end+1} = currentReading;

end

fclose(fid);

% Convert cell array to struct array
laser = [laser{:}];

fprintf('Read %d ROBOTLASER1 scans from %s\n', numel(laser), filename);

end
