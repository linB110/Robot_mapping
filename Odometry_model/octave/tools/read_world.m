function landmarks = read_world(filename)
    % Reads the world definition and returns a structure of landmarks.
    input = fopen(filename);
    if input == -1
        error('can not open file: %s', filename);
    end
    
    landmarks = struct('id', {}, 'x', {}, 'y', {});  
    idx = 1;
    
    while ~feof(input)
        line = fgetl(input);
        if ischar(line) && ~isempty(strtrim(line))
            data = strsplit(strtrim(line));
            if length(data) >= 3
                landmarks(idx).id = str2double(data{1});
                landmarks(idx).x  = str2double(data{2});
                landmarks(idx).y  = str2double(data{3});
                idx = idx + 1;
            end
        end
    end
    
    landmarks = landmarks(1:idx-1); 
    fclose(input);
end
