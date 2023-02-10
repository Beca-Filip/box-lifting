function V = computeValues(obj,computationTimes,derivativeOrder)
%COMPUTEVALUES calculates the SplineTrajectory and its derivatives' values 
%for the given input vector of computationTimes.
%
%   V = COMPUTEVALUES(obj,computationTimes,derivativeOrder)
%   Uses obj.knotTimes for which the spline coefficients were calculated, 
%   the obj.coefficients of the splines and the vector of input times
%   computationTimes for which the function value is to be calculated.
%   Returns the spline trajectory and its derivatives' values in a cell
%   array of size (derivativeOrder + 1). The values of the spline
%   trajectory and its derivatives are stacked in matrices of the size:
%   obj.dimension x Number of samples of computationTimes
%
%   V = COMPUTEVALUES(obj,computationTimes)
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

% Additional code for discerning symbolic from numeric times
% When times are symbolic, we cannot see to which piecewise polynomial the
% computation times belong, and so we cannot evaluate it
if strcmp(class(computationTimes), "casadi.SX") || strcmp(class(computationTimes), "casadi.MX")
    % Assumption evenly distributed knots and computation times
    numeric_knot_times = linspace(0, 1, obj.knotNumber);
    numeric_computation_times = linspace(0, 1, length(computationTimes));
% When times are numeric just keep them that way
else
    numeric_knot_times = obj.knotTimes;
    numeric_computation_times = computationTimes;
end

% Initialize total output
V = cell(1, 1+derivativeOrder);


% For all obj.dimension
for dd = 1 : obj.dimension
    
    % Intermediate output
    allValueAndDerivative = [];
    
    % For all obj.knotTimes before last knot
    for jj = 2 : obj.knotNumber  

%         % All timesamples less than current knot for dimension dd
%         valueAndDerivative = obj.computePolynomial(obj.coefficients((jj-2)*(obj.degree + 1)+1:(jj-1)*(obj.degree + 1), dd), ...
%                         computationTimes(computationTimes >= obj.knotTimes(jj-1) & computationTimes < obj.knotTimes(jj)),...
%                         derivativeOrder);
        % All timesamples less than current knot for dimension dd
        valueAndDerivative = obj.computePolynomial(obj.coefficients((jj-2)*(obj.degree + 1)+1:(jj-1)*(obj.degree + 1), dd), ...
                        computationTimes(find(numeric_computation_times >= numeric_knot_times(jj-1) & numeric_computation_times < numeric_knot_times(jj))),...
                        derivativeOrder);
        
        % Stacked time samples
        allValueAndDerivative = [allValueAndDerivative, valueAndDerivative];
    end
    
    % For all obj.knotTimes after last knot
%     % All timesamples less than current knot
%     valueAndDerivative = obj.computePolynomial(obj.coefficients(((obj.knotNumber - 1)-1)*(obj.degree + 1)+1:(obj.knotNumber - 1)*(obj.degree + 1), dd),...
%                                        computationTimes(computationTimes >= obj.knotTimes(obj.knotNumber)),...
%                                        derivativeOrder);
    % All timesamples less than current knot
    valueAndDerivative = obj.computePolynomial(obj.coefficients(((obj.knotNumber - 1)-1)*(obj.degree + 1)+1:(obj.knotNumber - 1)*(obj.degree + 1), dd),...
                                       computationTimes(find(numeric_computation_times >= numeric_knot_times(obj.knotNumber))),...
                                       derivativeOrder);
    % Stacked time samples
    allValueAndDerivative = [allValueAndDerivative, valueAndDerivative];
    
    % Stack the output by derivatives
    for derord = 0 : derivativeOrder
        % Add samples into rows of V{derord}
        V{derord+1} = [V{derord+1}; allValueAndDerivative(derord+1, :)];
    end
end

end