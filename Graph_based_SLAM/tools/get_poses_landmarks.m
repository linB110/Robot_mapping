function [poses, landmarks] = get_poses_landmarks(g)
% GET_POSES_LANDMARKS Extract offsets of pose and landmark states

poses = [];
landmarks = [];

keys = fieldnames(g.idLookup);

for i = 1:numel(keys)
    value = g.idLookup.(keys{i});

    dim = value.dimension;
    offset = value.offset;

    if dim == 3
        poses = [poses; offset];
    elseif dim == 2
        landmarks = [landmarks; offset];
    end
end

end
