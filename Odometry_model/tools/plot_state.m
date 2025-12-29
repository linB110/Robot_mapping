function plot_state(mu, landmarks, timestep, z)
%PLOT_STATE Visualizes the robot in the map and saves a PNG.

figure(1);
clf;
hold on;
grid on;

L = struct2cell(landmarks);

plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'k+', 'MarkerSize', 10, 'LineWidth', 2);

for i = 1:numel(z)
    id = z(i).id;
    mX = landmarks(id).x;
    mY = landmarks(id).y;
    line([mu(1), mX], [mu(2), mY], 'Color', 'b', 'LineWidth', 1);
end

drawrobot(mu(1:3), 'r', 3, 0.3, 0.3);

xlim([-2, 12]);
ylim([-2, 12]);

outDir = fullfile('..', 'plots');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

filename = fullfile(outDir, sprintf('odom_%03d.png', timestep));
exportgraphics(gca, filename); 
hold off;
end
