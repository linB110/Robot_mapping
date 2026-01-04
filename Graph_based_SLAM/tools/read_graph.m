function graph = read_graph(filename)
% READ_GRAPH Read a g2o file describing a 2D SLAM instance (MATLAB 2025 compatible)

fid = fopen(filename, 'r');
if fid == -1
    error('Cannot open file: %s', filename);
end

graph = struct( ...
    'x', [], ...
    'edges', [], ...
    'idLookup', struct() ...
);

disp('Parsing File');

while true
    ln = fgetl(fid);
    if ~ischar(ln)
        break;
    end

    tokens = strsplit(strtrim(ln));
    double_tokens = str2double(tokens);

    tk = 2;

    if strcmp(tokens{1}, 'VERTEX_SE2')
        id = int32(double_tokens(tk)); tk = tk + 1;
        values = double_tokens(tk:tk+2)'; tk = tk + 3;

        key = num2str(id);
        graph.idLookup.(key) = struct( ...
            'offset', length(graph.x), ...
            'dimension', length(values) ...
        );

        graph.x = [graph.x; values];

    elseif strcmp(tokens{1}, 'VERTEX_XY')
        id = int32(double_tokens(tk)); tk = tk + 1;
        values = double_tokens(tk:tk+1)'; tk = tk + 2;

        key = num2str(id);
        graph.idLookup.(key) = struct( ...
            'offset', length(graph.x), ...
            'dimension', length(values) ...
        );

        graph.x = [graph.x; values];

    elseif strcmp(tokens{1}, 'EDGE_SE2')
        fromId = int32(double_tokens(tk)); tk = tk + 1;
        toId   = int32(double_tokens(tk)); tk = tk + 1;

        measurement = double_tokens(tk:tk+2)'; tk = tk + 3;
        uppertri    = double_tokens(tk:tk+5)'; tk = tk + 6;

        information = [ ...
            uppertri(1), uppertri(2), uppertri(3); ...
            uppertri(2), uppertri(4), uppertri(5); ...
            uppertri(3), uppertri(5), uppertri(6) ...
        ];

        graph.edges(end+1) = struct( ...
            'type', 'P', ...
            'from', fromId, ...
            'to', toId, ...
            'measurement', measurement, ...
            'information', information ...
        );

    elseif strcmp(tokens{1}, 'EDGE_SE2_XY')
        fromId = int32(double_tokens(tk)); tk = tk + 1;
        toId   = int32(double_tokens(tk)); tk = tk + 1;

        measurement = double_tokens(tk:tk+1)'; tk = tk + 2;
        uppertri    = double_tokens(tk:tk+2)'; tk = tk + 3;

        information = [ ...
            uppertri(1), uppertri(2); ...
            uppertri(2), uppertri(3) ...
        ];

        graph.edges(end+1) = struct( ...
            'type', 'L', ...
            'from', fromId, ...
            'to', toId, ...
            'measurement', measurement, ...
            'information', information ...
        );
    end
end

fclose(fid);

% setup the index into the state vector
disp('Preparing helper structs');

for eid = 1:length(graph.edges)
    fromKey = num2str(graph.edges(eid).from);
    toKey   = num2str(graph.edges(eid).to);

    graph.edges(eid).fromIdx = graph.idLookup.(fromKey).offset + 1;
    graph.edges(eid).toIdx   = graph.idLookup.(toKey).offset + 1;
end

end
