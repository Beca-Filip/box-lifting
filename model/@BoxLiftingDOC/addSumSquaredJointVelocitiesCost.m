function obj = addSumSquaredJointVelocitiesCost(obj)

% Quantity to optimize:
quantity = obj.dq;
% Dimension of quantity
dimQuantity = size(quantity, 1);

% If in constant-weight mode
if strcmp(obj.parameter_mode, obj.parameter_mode1)
    % Constant cost:
    theCost = sum(sum(quantity.^2, 2) ./ obj.splineEvaluationNumber) ./ dimQuantity;

% If in variable-weight mode
elseif strcmp(obj.parameter_mode, obj.parameter_mode2)
    % Time-varying cost:    
    % Make number of parameters the size of the evaluation
    omega = obj.opti.parameter(1, obj.splineEvaluationNumber);
    % Add the time varying parameters to the matrix of parameters
    obj.omega_t = vertcat(obj.omega_t, omega);
    % Create time-weighted quantity:
    omega_times_quantity = (quantity .* repmat(omega, [dimQuantity, 1]));
    % Cost:
    theCost = sum(sum(omega_times_quantity.^2, 2) ./ obj.splineEvaluationNumber) ./ dimQuantity;
end

% Log the cost
obj.sumSquaredJointVelocities = theCost;

% Add the cost function to the vector:
obj.costFunctionVector = vertcat(obj.costFunctionVector, theCost);
end