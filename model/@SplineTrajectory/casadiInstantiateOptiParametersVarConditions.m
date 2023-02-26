function [obj, opti] = casadiInstantiateOptiParametersVarConditions(obj, numericSplineTrajectory, opti)
%CASADIINSTANTIATEOPTIPARAMETERSVARCONDITIONS returns an casadi.Opti object where the
%SplineTrajectory parameters have been instantiated with numerical values copied from 
%a numeric spline trajectory.
%The knotValues and the boundaryConditions at the final knot are treated as
%variables.
%
%   opti = CASADIINSTANTIATEOPTIPARAMETERS(obj, numericsplineTrajectory, opti)
%   
    % Set initial guess
    opti.set_initial(obj.knotValues, numericSplineTrajectory.knotValues);
    
    % Set values for the opti parameters
    opti.set_value(obj.knotTimes, numericSplineTrajectory.knotTimes);
    % Set values for all boundary conditions
    for bndCnd = 1 : length(obj.boundaryConditions)
        % If boundary condition is at the first knot, treat as parameter
        if obj.boundaryConditions.KnotOfCondition == 1
            opti.set_value(obj.boundaryConditions(bndCnd, 1).Value, numericSplineTrajectory.boundaryConditions(bndCnd, 1).Value);
        % Otherwise treat as variable
        else
            opti.set_initial(obj.boundaryConditions(bndCnd, 1).Value, numericSplineTrajectory.boundaryConditions(bndCnd, 1).Value);
        end
            
    end
    % Computation times
    opti.set_value(obj.currentEvaluatedTimes, numericSplineTrajectory.currentEvaluatedTimes);
end