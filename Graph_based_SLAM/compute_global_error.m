% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if strcmp(edge.type, 'P') 

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z_ij = v2t(edge.measurement);
    x_diff = (x1\x2);
    e_ij = t2v(Z_ij \ x_diff);

    e = e_ij' * edge.information * e_ij;

    % accumulate error
    Fx = Fx + e;

  % pose-landmark constraint
  elseif strcmp(edge.type, 'L') 
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    
    X  = v2t(x);
    Ri = X(1:2,1:2);
    
    e_il = Ri' * (l - x(1:2)) - edge.measurement;          
    Fx = Fx + e_il' * edge.information * e_il;
  end

end
