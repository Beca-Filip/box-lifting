function obj = addSumSquaredWristVelocityCost(obj)

% Cost:
obj.sumSquaredWristVelocity = sum(sum(obj.v_wri.^2, 2) ./ obj.splineEvaluationNumber) ./ 2;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, obj.sumSquaredWristVelocity);
end