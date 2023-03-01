function obj = setParametrizedCostFunction(obj)

% Number of costs
nCosts = length(obj.costFunctionVector);

% If in constant-weight mode
if strcmp(obj.parameter_mode, obj.parameter_mode1)

    % If only 1 cf
    if nCosts == 1
        % Set the parameter omega to 1
        obj.omega = 1;
        % Create the compound cost function
        obj.compoundCostFunction = obj.costFunctionVector;
        % Minimize this single cf
        obj.opti.minimize(obj.compoundCostFunction);
    else
        % Allocate the parameter omega
        obj.omega = obj.opti.parameter(nCosts, 1);
        % Create the compound cost function
        obj.compoundCostFunction = obj.omega.' * obj.costFunctionVector;
        % Minimize the compound cost funciton
        obj.opti.minimize(obj.compoundCostFunction);
    end
    
% If in variable-weight mode
elseif strcmp(obj.parameter_mode, obj.parameter_mode2)
    
    % Create the compound cost function
    obj.compoundCostFunction = sum(obj.costFunctionVector);
    % Minimize this single cf
    obj.opti.minimize(obj.compoundCostFunction);
    
end

end