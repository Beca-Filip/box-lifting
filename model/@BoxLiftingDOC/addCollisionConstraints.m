function obj = addCollisionConstraints(obj)

% Define a tolerance for this constraint
constraintTol = 3e-2;   % 0.03 m = 3cm

% Constraint:
obj.collisionConstraints = -vertcat(obj.distBodyToBox(:), obj.distBoxToTable(:)) - constraintTol;

% Set opti object to be subject to
obj.opti.subject_to(obj.collisionConstraints <= 0);
end