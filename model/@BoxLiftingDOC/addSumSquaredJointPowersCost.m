function obj = addSumSquaredJointPowersCost(obj)

% Cost:
obj.sumSquaredJointPowers = sum(sum((obj.tau .* obj.dq).^2, 2) ./ obj.splineEvaluationNumber) ./ obj.splineDimension;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, obj.sumSquaredJointPowers);
end