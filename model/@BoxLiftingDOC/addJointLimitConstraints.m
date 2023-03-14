function obj = addJointLimitConstraints(obj)

% Define a tolerance for this constraint
constraintTol = 2e-2;   % 0.02 radians = 1.12 degrees

% Constraint 1: Joint lower limit constraints
obj.jointLowerLimitConstraints = ...
-obj.q(:) + repmat(obj.casadiHumanModel.LowerJointLimits, [obj.splineEvaluationNumber, 1]) - constraintTol;

% Constraint 2: Joint upper limit constraints
obj.jointUpperLimitConstraints =  ...
obj.q(:) - repmat(obj.casadiHumanModel.UpperJointLimits, [obj.splineEvaluationNumber, 1]) - constraintTol;

% Set opti object to be subject to
obj.opti.subject_to(obj.jointLowerLimitConstraints <= 0);
obj.opti.subject_to(obj.jointUpperLimitConstraints <= 0);
end