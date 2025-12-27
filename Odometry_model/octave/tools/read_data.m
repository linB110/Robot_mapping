function data = read_data(filename)
% Reads the odometry and sensor readings from a file.
% Output:
%   data.timestep(i).odometry: struct with fields r1, t, r2
%   data.timestep(i).sensor(j): struct with fields id, range, bearing

fid = fopen(filename, 'r');
if fid == -1
    error('can not open file: %s', filename);
end

data = struct();
data.timestep = struct('odometry', {}, 'sensor', {});

first = true;
odom = struct();
sensor = struct('id', {}, 'range', {}, 'bearing', {});

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line); break; end
    line = strtrim(line);
    if isempty(line); continue; end

    arr = strsplit(line);
    type = strtrim(arr{1});

    if strcmp(type, 'ODOMETRY')
        % last timestep
        if ~first
            data.timestep(end+1).odometry = odom; %#ok<AGROW>
            data.timestep(end).sensor = sensor;
            sensor = struct('id', {}, 'range', {}, 'bearing', {});
        end
        first = false;

        odom.r1 = str2double(arr{2});
        odom.t  = str2double(arr{3});
        odom.r2 = str2double(arr{4});

    elseif strcmp(type, 'SENSOR')
        reading = struct();
        reading.id      = str2double(arr{2});
        reading.range   = str2double(arr{3});
        reading.bearing = str2double(arr{4});
        sensor(end+1) = reading; %#ok<AGROW>
    end
end

if ~first
    data.timestep(end+1).odometry = odom; %#ok<AGROW>
    data.timestep(end).sensor = sensor;
end

fclose(fid);
end
