function landmarks = read_world(filename)

    input = fopen(filename);

    landmarks = struct('id', {}, 'x', {}, 'y', {});

    while ~feof(input)
        line = fgetl(input);
        if ~ischar(line)
            continue;
        end

        data = strsplit(strtrim(line), ' ');
        if numel(data) < 3
            continue;
        end

        landmark.id = str2double(data{1});
        landmark.x  = str2double(data{2});
        landmark.y  = str2double(data{3});

        landmarks(end+1) = landmark; %#ok<AGROW>
    end

    fclose(input);
end