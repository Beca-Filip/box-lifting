function obj = addJointLimitConstraints(obj)

% Define a tolerance for this constraint
constraintTol = 1e-2;   % 0.01 radians = 0.56 degrees

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