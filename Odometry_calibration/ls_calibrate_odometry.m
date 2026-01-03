% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, utheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
  % initial solution (the identity transformation)
  X = eye(3); 
  
  % Z = [ scan mating (x, y theta)  odom (x, y, theta) ]

  % TODO: initialize H and b of the linear system
  % state vector = [x11 x12 x13 x21 x22 x23 x31 x32 x33]
  H = zeros(9,9);
  b_T = zeros(1,9);

  % initialize infomation matrix = identity
  omega = eye(3); 

  % TODO: loop through the measurements and update H and b
  % You may call the functions error_function and jacobian, see below
  % We assume that the information matrix is the identity.
  for i = 1 : size(Z,1)
      e_i = error_function(i, X, Z);
      J_i = jacobian(i, Z);
      
      % H = Σ Hi = Σ JiT Ω Ji  /  b = Σ bi = Σ eiΩJi
      H = H + J_i' * omega * J_i; 
      b_T = b_T + (e_i' * omega * J_i);
  end
  
  % transpose b_T back to b
  b = b_T';
  
  % TODO: solve and update the solution
  % delta_X is a 9x9 matrix
  % but calibration matrix X is 3x3
  delta_x = -inv(H)*b;
  X = X + [delta_x(1:3)'; delta_x(4:6)'; delta_x(7:9)'];
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
  % TODO compute the error of each measurement
  % Z(i, 1:3) is row vector
  e = Z(i, 1:3)' - X * Z(i, 4:6)';
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement

% from silde : Ji = -[ux uy uθ 0  0  0  0  0  0
%                     0  0  0  ux uy uθ
%                     0  0  0  0  0  0  ux uy uθ]
function J = jacobian(i, Z)
    J = zeros(3, 9);
    % TODO compute the Jacobian
    odom_value = Z(i, 4:6);

    J(1, 1:3) = -odom_value;
    J(2, 4:6) = -odom_value;
    J(3, 7:9) = -odom_value;
end
