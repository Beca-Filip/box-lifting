function obj = addSumSquaredJointTorquesCost(obj)

% Cost:
obj.sumSquaredJointTorques = sum(sum(obj.tau.^2, 2) ./ obj.splineEvaluationNumber) ./ obj.splineDimension;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, obj.sumSquaredJointTorques);
end