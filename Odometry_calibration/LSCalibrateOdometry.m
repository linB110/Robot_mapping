more off;
clear all;
close all;

% add tools directory
addpath('tools')

% load the odometry measurements
odom_motions = dlmread('../data/odom_motions','',5,0);

% the motions as they are estimated by scan-matching
scanmatched_motions = dlmread('../data/scanmatched_motions','',5,0);

% create our measurements vector z
z = [scanmatched_motions odom_motions];

% perform the calibration
X = ls_calibrate_odometry(z);
disp('calibration result'); disp(X);

% apply the estimated calibration parameters
calibrated_motions = apply_odometry_correction(X, odom_motions);

% compute the current odometry trajectory, the scanmatch result, and the calibrated odom
odom_trajectory = compute_trajectory(odom_motions);
scanmatch_trajectory = compute_trajectory(scanmatched_motions);
calibrated_trajectory = compute_trajectory(calibrated_motions);

% plot the trajectories
figure; 
hold on; 
axis equal;

plot(odom_trajectory(:,1), odom_trajectory(:,2), 'DisplayName','Uncalibrated Odometry');
plot(scanmatch_trajectory(:,1), scanmatch_trajectory(:,2), 'DisplayName','Scan-Matching');
plot(calibrated_trajectory(:,1), calibrated_trajectory(:,2), 'DisplayName','Calibrated Odometry');

legend('Location','northeast');

print('-dpng','../plots/odometry-calibration.png')

