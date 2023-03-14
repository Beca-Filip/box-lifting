function obj = addCollisionConstraints(obj)

% Define a tolerance for this constraint
constraintTol = 8e-2;   % 0.08 m = 8cm

% Constraint:
obj.collisionConstraints = -vertcat(obj.distBodyToBox(:), obj.distBoxToTable(:)) - constraintTol;

% Set opti object to be subject to
obj.opti.subject_to(obj.collisionConstraints <= 0);
end