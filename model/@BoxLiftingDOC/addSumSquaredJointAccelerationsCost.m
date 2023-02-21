function obj = addSumSquaredJointAccelerationsCost(obj)

% Cost:
obj.sumSquaredJointAccelerations = sum(sum(obj.ddq.^2, 2) ./ obj.splineEvaluationNumber) ./ obj.splineDimension;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, obj.sumSquaredJointAccelerations);
end