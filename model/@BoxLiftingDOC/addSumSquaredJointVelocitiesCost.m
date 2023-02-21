function obj = addSumSquaredJointVelocitiesCost(obj)

% Cost:
obj.sumSquaredJointVelocities = sum(sum(obj.dq.^2, 2) ./ obj.splineEvaluationNumber) ./ obj.splineDimension;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, obj.sumSquaredJointVelocities);
end