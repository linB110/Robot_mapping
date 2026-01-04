% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  % Compute the homogenous transformations
  X1 = v2t(x1);
  X2 = v2t(x2);
  Z = v2t(z);
  
  % Compute the error in the pose
  e = t2v( Z \ (X1 \ X2) ); 
    
  % formula of compute the Jacobians
  % ∂e/∂X = [∂ex/∂x ∂ex/∂y ∂ex/∂θ
  %          ∂ey/∂x ∂ey/∂y ∂ey/∂θ
  %          ∂eθ/∂x ∂eθ/∂y ∂eθ/∂θ]

  % e = [ex; ey; eθ]
  % et = [ex; ey] = Rij^T * Ri^T * [xj-xi; yj-yi]
  % eθ = [Rij^T * Ri^T * Rj] = [θj-θi-θij]
  
  % Rotation matrices
  Rij = Z(1:2, 1:2);
  Ri = X1(1:2, 1:2);
  Rj = X2(1:2, 1:2);
  
  % translation vectors
  ti = [x1(1); x1(2)];
  tj = [x2(1); x2(2)];
  tij = [z(1); z(2)];

  % construct et, eθ
  et = Rij' * (Ri'*(tj - ti) - tij);
  e_theta = Rij'*Ri'*Rj;

  % Compute the Jacobians
  A = zeros(3, 3);
  B = zeros(3, 3);
  
  % Jacobian with respect to x1
  A(1:2, 1:2) = -Rij' * Ri';

  theta_i = x1(3);
  A(1:2, 3) = Rij' * [-sin(theta_i) cos(theta_i); -cos(theta_i) -sin(theta_i)] * (tj - ti);
  A(3, 3) = -1;

  % Jacobian with respect to x2
  B(1:2, 1:2) = Rij' * Ri';
  B(3, 3) = 1;
end
