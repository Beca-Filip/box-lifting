function obj = addSumSquaredWristAccelerationCost(obj)

% Cost:
obj.sumSquaredWristAcceleration = sum(sum(obj.a_wri.^2, 2) ./ obj.splineEvaluationNumber) ./ 2;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, obj.sumSquaredWristAcceleration);
end