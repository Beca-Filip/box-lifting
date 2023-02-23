function obj = addTorqueLimitConstraints(obj)

% Constraint: Torque lower limit constraints
obj.torqueLowerLimitConstraints = ...
-obj.tau(:) + repmat(obj.casadiHumanModel.LowerTorqueLimits, [obj.splineEvaluationNumber, 1]);

% Constraint: Torque upper limit constraints
obj.torqueUpperLimitConstraints =  ...
obj.tau(:) - repmat(obj.casadiHumanModel.UpperTorqueLimits, [obj.splineEvaluationNumber, 1]);

% Set opti object to be subject to
obj.opti.subject_to(obj.torqueLowerLimitConstraints <= 0);
obj.opti.subject_to(obj.torqueUpperLimitConstraints <= 0);
end