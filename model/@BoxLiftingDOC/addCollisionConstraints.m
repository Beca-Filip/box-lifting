function obj = addCollisionConstraints(obj)

% Constraint:
obj.collisionConstraints = -vertcat(obj.distBodyToBox(:), obj.distBoxToTable(:));

% Set opti object to be subject to
obj.opti.subject_to(obj.collisionConstraints <= 0);
end