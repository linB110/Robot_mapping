function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
Q_t = 0.1 * eye(2);

% process each particle
for i = 1:numParticles
  robot = particles(i).pose;
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(l).observed == false)

      % TODO: Initialize its position based on the measurement and the current robot pose:
      robot_x = robot(1);
      robot_y = robot(2);
      robot_theta = robot(3);

      % Initialize landmark observation
      particles(i).landmarks(l).mu = [robot_x + z(j).range * cos(normalize_angle(z(j).bearing + robot_theta));
                                     robot_y + z(j).range * sin(normalize_angle(z(j).bearing + robot_theta))];

      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the EKF for this landmark
      % Initialize the covariance to identity
      % Compute covariance
      H_inv = inv(H);
      particles(i).landmarks(l).sigma = H_inv * Q_t * H_inv';

      % initalize weight
      particles(i).weight = 1 / numParticles;

      % Indicate that this landmark has been observed
      particles(i).landmarks(l).observed = true;

    else

      % get the expected measurement
      [expectedZ, H] = measurement_model(particles(i), z(j));

      % TODO: compute the measurement covariance
      Q = H * particles(i).landmarks(l).sigma * H' + Q_t;
      
      % TODO: calculate the Kalman gain
      K = particles(i).landmarks(l).sigma * H' / Q;
       
      % TODO: compute the error between the z and expectedZ (remember to normalize the angle)
      diff_z = [z(j).range - expectedZ(1); normalize_angle(z(j).bearing - expectedZ(2))];
      diff_z(2) = normalize_angle(diff_z(2));

      % TODO: update the mean and covariance of the EKF for this landmark
      particles(i).landmarks(l).mu = particles(i).landmarks(l).mu + K * diff_z; 
      particles(i).landmarks(l).sigma = (eye(2) - K * H) * particles(i).landmarks(l).sigma;

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
      dim = length(diff_z);  

      norm_const = 1 / sqrt((2*pi)^dim * det(Q));
      exponent   = -0.5 * diff_z' * (Q \ diff_z);
        
      particles(i).weight = norm_const * exp(exponent);

    end

  end % measurement loop
end % particle loop

end
