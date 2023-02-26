function obj = addSumSquaredJointJerksCost(obj)

% Cost:
obj.sumSquaredJointJerks = sum(sum(obj.dddq.^2, 2) ./ obj.splineEvaluationNumber) ./ obj.splineDimension;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, obj.sumSquaredJointJerks);
end