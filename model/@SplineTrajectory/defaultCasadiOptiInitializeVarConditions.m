function [obj, opti] = defaultCasadiOptiInitializeVarConditions(opti, dimension, degree, knotNumber, evaluationNumber)
%DEFAULTCASADIOPTIINITIALIZEVARCONDITIONS returns a SplineTrajectory object with default
%casadi opti initializers, but treats boundary conditions at the last
%sample as variables.
%
%   [obj, opti] = defaultCasadiOptiInitialize(opti, dimension, degree, knotNumber, evaluationNumber)
%   Takes in the dimension of the trajectory, degree of the spline
%   polynomials, the number of knots in the spline, and the number of
%   points along which the spline is evaluated.

%% Variables
% The variables of the trajectory
knotValues = opti.variable(dimension, knotNumber);

%% Parameters
% Get all default parameter values
knotTimes = opti.parameter(1, knotNumber);

% Get the default boundary conditions
[boundaryConditions, opti] = SplineTrajectory.getDefaultCasadiOptiBoundaryConditions(opti, dimension, degree, knotNumber);

% Create and return the spline object
obj = SplineTrajectory(knotTimes, knotValues, degree, boundaryConditions);

% The times at which the trajectory is evaluated
computationTimes = opti.parameter(1, evaluationNumber);

% Compute the values
obj = obj.computeValuesAndStore(computationTimes, degree);
end

