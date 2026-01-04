% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error
  % t_i = robot translation
  t_i = [x(1); x(2)];
  X = v2t(x);
  Ri = X(1:2, 1:2);

  % error function = Ri^T * (xj - ti) - Zij
  e = Ri' * (l - t_i) - z;
  
  % TODO compute Jacobians of the error
  
  % initialize Jacobians
  A = zeros(2,3);
  B = zeros(2,2);
  
  % ∂e/∂X(x,y)  
  A(1:2, 1:2) = -Ri';
  
  % ∂e/∂θ 
  theta_i = x(3);
  A(1:2, 3) = [-sin(theta_i) cos(theta_i); -cos(theta_i) -sin(theta_i)] * (l - t_i);
  

  % ∂e/∂l
  B(1:2, 1:2) = Ri';
end
