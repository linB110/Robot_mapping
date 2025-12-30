function data = read_data(filename)

    input = fopen(filename);

    data.timestep = struct('odometry', {}, 'sensor', {});

    first = true;
    odom = struct; 
    sensor = struct('id', {}, 'range', {}, 'bearing', {});

    while ~feof(input)
        line = fgetl(input);
        if ~ischar(line)
            continue;
        end

        arr = strsplit(strtrim(line), ' ');
        type = arr{1};

        if strcmp(type, 'ODOMETRY')
            if ~first
                data.timestep(end+1).odometry = odom; %#ok<AGROW>
                data.timestep(end).sensor = sensor;
            end
            first = false;
            odom.r1 = str2double(arr{2});
            odom.t  = str2double(arr{3});
            odom.r2 = str2double(arr{4});
            sensor = struct('id', {}, 'range', {}, 'bearing', {});
        elseif strcmp(type, 'SENSOR')
            reading.id      = str2double(arr{2});
            reading.range   = str2double(arr{3});
            reading.bearing = str2double(arr{4});
            sensor(end+1) = reading; %#ok<AGROW>
        end
    end

    fclose(input);
end
