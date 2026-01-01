function points = robotlaser_as_cartesian(rl, maxRange, subsample)
% Convert robot laser scan to Cartesian points (MATLAB compatible)

% --- default arguments (MATLAB style) ---
if nargin < 2 || isempty(maxRange)
    maxRange = 15;
end
if nargin < 3 || isempty(subsample)
    subsample = false;
end

numBeams = length(rl.ranges);
maxRange = min(maxRange, rl.maximum_range);

% apply the max range
idx = (rl.ranges < maxRange) & (rl.ranges > 0);

if subsample
    idx(2:2:end) = false;
end

% compute angles
angles_all = linspace( ...
    rl.start_angle, ...
    rl.start_angle + (numBeams-1)*rl.angular_resolution, ...
    numBeams);

angles = angles_all(idx);

% homogeneous coordinates
points = [ ...
    rl.ranges(idx) .* cos(angles); ...
    rl.ranges(idx) .* sin(angles); ...
    ones(1, length(angles)) ];

% apply the laser offset
transf = v2t(rl.laser_offset);
points = transf * points;

end
