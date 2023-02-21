function obj = addFinalJointConstraints(obj)

% Constraint:
obj.finalJointConstraints = obj.q(:, end) - obj.casadiLiftingEnvironment.JointAnglesFinal;

% Set opti object to be subject to
obj.opti.subject_to(obj.finalJointConstraints == 0);
end