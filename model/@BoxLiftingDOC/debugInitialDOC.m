% STANDARD DEBUGGING SCRIPT
function ErrorCounter = debugInitialDOC(obj, varargin)
    
    % If argument passed it's the constraint tolerance
    DEFAULT_IPOPT_TOLERANCE = 1e-4;
    if nargin > 1
        tol = varargin{1};
    else
        tol = DEFAULT_IPOPT_TOLERANCE;
    end
    
    fprintf("Checking for errors...\n");
    ErrorCounter = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NANS and INFS
    %  Cost function vector
    % Cost function vector NANs
    if any(isnan(obj.opti.debug.value(obj.costFunctionVector, obj.opti.initial())), 'all')
        fprintf("Cost function vector contains nans.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Cost function vector INFS
    if any(isinf(obj.opti.debug.value(obj.costFunctionVector, obj.opti.initial())), 'all')
        fprintf("Cost function vector contains infs.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Jacobian of cost function vector NANs
    if any(isnan(obj.opti.debug.value(jacobian(obj.costFunctionVector, obj.opti.x), obj.opti.initial())), 'all')
        fprintf("Jacobian of cost function vector contains nans.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Jacobian of cost function vector INFs
    if any(isinf(obj.opti.debug.value(jacobian(obj.costFunctionVector, obj.opti.x), obj.opti.initial())), 'all')
        fprintf("Jacobian of cost function vector contains infs.\n");
        ErrorCounter = ErrorCounter+1;
    end

    % Total cost function
    % Cost function NAN
    if any(isnan(obj.opti.debug.value(obj.opti.f, obj.opti.initial())), 'all')    
        fprintf("Cost function contains nans.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Cost function INF
    if any(isinf(obj.opti.debug.value(obj.opti.f, obj.opti.initial())), 'all')    
        fprintf("Cost function contains infs.\n");
        ErrorCounter = ErrorCounter+1;
    end

    % Get hessian and gradient
    [Hf, gf] = hessian(obj.opti.f, obj.opti.x);

    % Cost function gradient NANs
    if any(isnan(obj.opti.debug.value(gf, obj.opti.initial())), 'all')    
        fprintf("Gradient of cost function contains nan.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Cost function gradient INF
    if any(isinf(obj.opti.debug.value(gf, obj.opti.initial())), 'all')    
        fprintf("Gradient of cost function contains infs.\n");
        ErrorCounter = ErrorCounter+1;
    end

    % Cost function hessian NANs
    if any(isnan(obj.opti.debug.value(Hf, obj.opti.initial())), 'all')    
        fprintf("Hessian of cost function contains nan.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Cost function hessian INF
    if any(isinf(obj.opti.debug.value(Hf, obj.opti.initial())), 'all')    
        fprintf("Hessian of cost function contains infs.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Constraint functions
    
    % Constraint function NAN
    if any(isnan(obj.opti.debug.value(obj.opti.g, obj.opti.initial())), 'all')    
        fprintf("Constraint function contains nans.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Constraint function INF
    if any(isinf(obj.opti.debug.value(obj.opti.g, obj.opti.initial())), 'all')    
        fprintf("Constraint function contains infs.\n");
        ErrorCounter = ErrorCounter+1;
    end
    
    % Constraint function Jacobian NAN
    if any(isnan(obj.opti.debug.value(jacobian(obj.opti.g, obj.opti.x), obj.opti.initial())), 'all')    
        fprintf("Jacobian of constraint function contains nans.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Constraint function INF
    if any(isinf(obj.opti.debug.value(jacobian(obj.opti.g, obj.opti.x), obj.opti.initial())), 'all')    
        fprintf("Jacobian of constraint function contains infs.\n");
        ErrorCounter = ErrorCounter+1;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Infeasibility
    
    % Check lower bound constraints
    if any(-obj.opti.debug.value(obj.opti.g, obj.opti.initial()) + obj.opti.debug.value(obj.opti.lbg, obj.opti.initial()) > tol, 'all')
        fprintf("Infeasibility: Standard form constraint lower bound transgressed.\n");
        ErrorCounter = ErrorCounter+1;
    end
    % Check upper bound constraints
    if any(obj.opti.debug.value(obj.opti.g, obj.opti.initial()) - obj.opti.debug.value(obj.opti.ubg, obj.opti.initial()) > tol, 'all')
        fprintf("Infeasibility: Standard form constraint upper bound transgressed.\n");
        ErrorCounter = ErrorCounter+1;        
    end
    
    % Identify which constraints
    % Get all the property names
    propertyNames = properties(obj);
    % Get all the property names containing constraints
    constraintNames = propertyNames(contains(propertyNames, "constraint", "IgnoreCase", true));
    % For every constraint name
    for numCons = 1 : length(constraintNames)
        
        % Get the current constraint
        cons = constraintNames{numCons};
        
        % Check if the given property is nonempty
        if ~isempty(obj.(cons))
            
            % Check its infeasibility
            if any(obj.opti.debug.value(obj.(cons), obj.opti.initial()) > tol, 'all')
                fprintf("\tInfeasibility: %s infeasible.\n", cons);
            end
        end
        
    end
    
    %%
    fprintf("Errors found: %d.\n", ErrorCounter);
end