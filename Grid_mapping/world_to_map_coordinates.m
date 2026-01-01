function [pntsMap] = world_to_map_coordinates(pntsWorld, gridSize, offset)
% Convert 2xN world coordinates to map indices (row,col)
% Map rows → y, Map cols → x
% pntsWorld: 2xN [x;y] in meters
% offset: [offsetX; offsetY]
% gridSize: meters per cell
% Output pntsMap: 2xN, [row; col] indices for MATLAB

cols = round((pntsWorld(1,:) - offset(1)) / gridSize) + 1;  % x → col
rows = round((pntsWorld(2,:) - offset(2)) / gridSize) + 1;  % y → row

pntsMap = [rows; cols];
end
