function obj = addCopLimitConstraints(obj)

% Constraint: COP lower limit constraints
obj.copLowerLimitConstraints = ...
-obj.cop(:) + repmat(obj.casadiHumanModel.HeelPosition(1) - 0.3 * obj.casadiHumanModel.LFOOT, [obj.splineEvaluationNumber, 1]);

% Constraint: COP upper limit constraints
obj.copUpperLimitConstraints =  ...
obj.cop(:) - repmat(obj.casadiHumanModel.ToePosition(1) + 0.3 * obj.casadiHumanModel.LFOOT, [obj.splineEvaluationNumber, 1]);

% Set opti object to be subject to
obj.opti.subject_to(obj.copLowerLimitConstraints <= 0);
obj.opti.subject_to(obj.copUpperLimitConstraints <= 0);
end