function obj = setCostFunctionParameters(obj, omega)

% Number of costs
nCosts = length(obj.costFunctionVector);

% If only 1 cf
if nCosts == 1
    % Omega should already be 1
    return
else
    % The value of omega needs to be set
    obj.opti.set_value(obj.omega, omega);
end

end