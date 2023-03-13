function obj = addInitialJointConstraints(obj)

% Define a tolerance for this constraint
constraintTol = 5e-3;   % 0.005 radians = 0.28 degrees

% Constraint:
obj.initialJointConstraints = [...
obj.q(:, 1) - obj.casadiLiftingEnvironment.JointAnglesInitial - constraintTol;
-obj.q(:, 1) + obj.casadiLiftingEnvironment.JointAnglesInitial - constraintTol;
];

% Set opti object to be subject to
obj.opti.subject_to(obj.initialJointConstraints <= 0);
end