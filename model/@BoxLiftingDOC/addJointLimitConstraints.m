function obj = addJointLimitConstraints(obj)
% Constraint 1: Joint lower limit constraints
obj.jointLowerLimitConstraints = ...
-obj.q(:) + repmat(obj.casadiHumanModel.LowerJointLimits, [obj.splineEvaluationNumber, 1]);

% Constraint 2: Joint upper limit constraints
obj.jointUpperLimitConstraints =  ...
obj.q(:) - repmat(obj.casadiHumanModel.UpperJointLimits, [obj.splineEvaluationNumber, 1]);

% Set opti object to be subject to
obj.opti.subject_to(obj.jointLowerLimitConstraints <= 0);
obj.opti.subject_to(obj.jointUpperLimitConstraints <= 0);
end