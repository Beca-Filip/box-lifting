function obj = computeValuesAndStore(obj,computationTimes,derivativeOrder)
%COMPUTEVALUESANDSTORE calculates the SplineTrajectory and its derivatives'
%values for the given input vector of computationTimes, and stores them in the
%appropriate object fields.
%
%   obj.currentEvaluatedTimes
%   obj.currentEvaluatedValuesAndDerivatives
%   obj.currentEvaluatedDerivatives
%
%   obj = COMPUTEVALUESANDSTORE(obj,computationTimes,derivativeOrder)
%   Uses obj.knotTimes for which the spline coefficients were calculated, 
%   the obj.coefficients of the splines and the vector of input times
%   computationTimes for which the function value is to be calculated.
%   Returns the spline trajectory and its derivatives' values in a cell
%   array of size (derivativeOrder + 1). The values of the spline
%   trajectory and its derivatives are stacked in matrices of the size:
%   obj.dimension x Number of samples of computationTimes
%
%   obj = COMPUTEVALUESANDSTORE(obj,computationTimes)
%   Assumes the derivative order is 0.

if nargin == 3
elseif nargin == 2
    derivativeOrder = 0;
else
    error("Invalid number of arguments.");
end

% Raise error if computationTimes and obj.knotTimes aren't of the same
% class/type
if ~strcmp(class(computationTimes), class(obj.knotTimes))
    error("computationTimes and obj.knotTimes must be of the same class/type.")
end

% Compute the values
V = obj.computeValues(computationTimes, derivativeOrder);

% Store the results
obj.currentEvaluatedTimes = computationTimes;
obj.currentEvaluatedValuesAndDerivatives = V;
obj.currentEvaluatedDerivatives = derivativeOrder;

end