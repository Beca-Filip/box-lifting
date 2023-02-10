function [obj, opti] = casadiInstantiateOptiParameters(obj, numericSplineTrajectory, opti)
%CASADIINSTANTIATEOPTIPARAMETERS returns an casadi.Opti object where the
%SplineTrajectory parameters have been instantiated with numerical
%values copied from a numeric spline trajectory.
%
%   opti = CASADIINSTANTIATEOPTIPARAMETERS(obj, numericsplineTrajectory, opti)
%   
    % Set initial guess
    opti.set_initial(obj.knotValues, numericSplineTrajectory.knotValues);
    
    % Set values for the opti parameters
    opti.set_value(obj.knotTimes, numericSplineTrajectory.knotTimes);
    % Set values for all boundary conditions
    for bndCnd = 1 : length(obj.boundaryConditions)
        opti.set_value(obj.boundaryConditions(bndCnd, 1).Value, numericSplineTrajectory.boundaryConditions(bndCnd, 1).Value);
    end
    % Computation times
    opti.set_value(obj.currentEvaluatedTimes, numericSplineTrajectory.currentEvaluatedTimes);
end