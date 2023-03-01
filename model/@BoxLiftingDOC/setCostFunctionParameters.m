function obj = setCostFunctionParameters(obj, omega)

% If in constant-weight mode
if strcmp(obj.parameter_mode, obj.parameter_mode1)
    
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

% If in variable-weight mode
elseif strcmp(obj.parameter_mode, obj.parameter_mode2)
    
    % The value of omega needs to be set
    obj.opti.set_value(obj.omega_t, omega);
    
end

end