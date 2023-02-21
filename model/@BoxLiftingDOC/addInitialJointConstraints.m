function obj = addInitialJointConstraints(obj)

% Constraint:
obj.initialJointConstraints = obj.q(:, 1) - obj.casadiLiftingEnvironment.JointAnglesInitial;

% Set opti object to be subject to
obj.opti.subject_to(obj.initialJointConstraints == 0);
end