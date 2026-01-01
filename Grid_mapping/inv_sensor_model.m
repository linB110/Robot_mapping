function [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = ...
    inv_sensor_model(map, scan, robPose, gridSize, offset, probOcc, probFree)

mapUpdate = zeros(size(map));

% Robot pose as homogeneous transform
robTrans = v2t(robPose);

% Robot pose in map frame
robPoseMapFrame = world_to_map_coordinates(robPose(1:2), gridSize, offset);
robPoseMapFrame(3) = robPose(3);

% Laser endpoints in robot frame
laserEndPnts = robotlaser_as_cartesian(scan, 30, false);

% Transform to world frame
laserEndPnts = robTrans * laserEndPnts;

% Transform to map frame
laserEndPntsMapFrame = world_to_map_coordinates( ...
    laserEndPnts(1:2,:), gridSize, offset);

freeCells = [];

mapH = size(map,1);
mapW = size(map,2);

% For each laser beam
for sc = 1:size(laserEndPntsMapFrame,2)

    x1 = robPoseMapFrame(1);
    y1 = robPoseMapFrame(2);
    x2 = laserEndPntsMapFrame(1, sc);
    y2 = laserEndPntsMapFrame(2, sc);

    % Bresenham ray tracing
    p1 = round([x1 y1]);
    p2 = round([x2 y2]);

    [X, Y] = bresenham([p1; p2]);


    freeCells = [freeCells, [X; Y]];
end

% Update free cells
for i = 1:size(freeCells,2)
    x = freeCells(1,i);
    y = freeCells(2,i);

    if x >= 1 && x <= mapW && y >= 1 && y <= mapH
        mapUpdate(y, x) = prob_to_log_odds(probFree);
    end
end

% Update occupied cells (laser endpoints)
for i = 1:size(laserEndPntsMapFrame,2)
    x = laserEndPntsMapFrame(1,i);
    y = laserEndPntsMapFrame(2,i);

    if x >= 1 && x <= mapW && y >= 1 && y <= mapH
        mapUpdate(y, x) = prob_to_log_odds(probOcc);
    end
end

end
