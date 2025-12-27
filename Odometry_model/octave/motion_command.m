function [x] = motion_command(x, u)
% Updates the robot pose according to the motion model
% x: 3x1 vector representing the robot pose [x; y; theta]
% u: struct containing odometry reading (r1, t, r2).
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% pose at t - 1
x_ori = x(1)
y_ori = x(2)
theta_ori = x(3)

% odometry reading at t
r1 = u.r1;
t  = u.t;
r2 = u.r2;

% update pose according to the motion represented by u
x_update = x_ori + t * cos(r1 + theta_ori)
y_update = y_ori + t * sin(r1 + theta_ori)
theta_update = theta_ori + r2 + r1

% normalize theta by calling normalize_angle for x(3)
theta_update = normalize_angle(theta_update)

x = [x_update, y_update, theta_update]

end
