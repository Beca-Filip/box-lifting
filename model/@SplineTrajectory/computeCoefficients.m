function coefficients = computeCoefficients(obj)
%COMPUTECOEFFICIENTS computes the coefficients of the SplineTrajectory.
%   coefficients = COMPUTECOEFFICIENTS() returns the coefficients of the
%   SplineTrajectory.
%
%   coefficients = COMPUTECOEFFICIENTS(obj)
%   gives back the coefficients of polynomials of a given order obj.degree
%   approximating the function f(obj.knotTimes) = obj.knotValues given by 
%   the vector of knot points obj.knotTimes and the corresponding function 
%   values obj.knotValues. A representation of order obj.degree requires 
%   another obj.degree-1 boundary conditions for the coefficients to be 
%   uniquely defined. In this implementation boundary conditions must be 
%   passed.
%   The variable obj.boundaryConditions is an array variable of size (obj.degree-1)x3 
%   where each row represents one equality condition upon the spline 
%   representation of the function, or one of its first obj.degree derivatives. 
%   Each row is represented in the following format:
%   [derivativeOrder, conditionTime, conditionValue1, ..., conditionValue(obj.dimesion)]:
%   -derivativeOrder: Order of the derivative upon which the condition is 
%   placed, it can range from 0 to obj.degree.
%   -conditionTime: the value of time (similar to obj.knotTimes) at which 
%   the condition is placed
%   -conditionValue1, ...,conditionValue(obj.dimesion) : the value which 
%   the derivative of derivativeOrder should take at x_val
%   
%   Note: Compatible with CASADI variables.

%   Author: Filip Becanovic
%   Last Modified: 27.01.2022.

% ================= Input verification and error checks ================= %

% Number of polynomials
n = length(obj.knotTimes) - 1;

% Number of boundary conditions
[nBndCnd, ~] = size(obj.boundaryConditions);

% ======================================================================= %

% ======================== Coded functionalities ======================== %

% Matrix A
A = zeros((obj.degree+1) * n, (obj.degree+1) * n, class(obj.knotTimes));

% Matrix b
b = zeros((obj.degree+1) * n, 1, 'logical');

% For each polynomial up to the next to last one add interior and exterior 
% knot-point equalities
for i = 1 : n-1
    
    % Conditions upon the endpoints of the polynomials
    % For each coefficient
    for j = 1 : obj.degree+1
        
        % Equate the value of the i-th polynomial at the i-th time sample 
        % with the true value
        A((obj.degree+1) * (i-1) + 1, (obj.degree+1) * (i-1) + j) = obj.knotTimes(i).^(j-1);
        
        % Equate the value of the i-th polynomial at the (i+1)-th time
        % sample with the value of the (i+1)-th polynomial at the same time sample
        A((obj.degree+1) * (i-1) + 2, (obj.degree+1) * (i-1) + j) = obj.knotTimes(i+1).^(j-1);    
        A((obj.degree+1) * (i-1) + 2, (obj.degree+1) * i + j) = - obj.knotTimes(i+1).^(j-1);
    end
    
    % Equate the value of the i-th polynomial at the i-th time sample 
    % with the true value
    b((obj.degree+1) * (i-1) + 1) = 1;
end

% Add equalities for the last polynomial
for j = 1 : obj.degree+1

    % Equate the value of the n-th polynomial at the n-th time sample 
    % with the true value
    A((obj.degree+1) * (n-1) + 1, (obj.degree+1) * (n-1) + j) = obj.knotTimes(n).^(j-1);

    % Equate the value of the n-th polynomial at the (n+1)-th time
    % sample with the sample value
    A((obj.degree+1) * (n-1) + 2, (obj.degree+1) * (n-1) + j) = obj.knotTimes(n+1).^(j-1);
end
% Equate the value of the inth polynomial at the n-th time sample with the 
% true value
b((obj.degree+1) * (n-1) + 1) = 1;
b((obj.degree+1) * (n-1) + 2) = 1;


% If the order of the polynomial is greater than one, add boundary
% conditions and derivative matching at interior knot points
if obj.degree > 1

% For each polynomial up to the next-to last one add interior point
% derivative equalities
for i = 1 : n - 1
    
    % For each derivative up to obj.degree-1
    for d = 1 : obj.degree - 1

        % Conditions upon the derivatives at the endpoints of the polynomial
        % For each coefficient
        for j = d+1 : obj.degree+1
        
            % Equate the value of the d-th derivative of the i-th 
            % polynomial at the (i+1)-th time sample with the value of the
            % derivative of the (i+1)-th polynomial at the same time sample
            A((obj.degree+1) * (i-1) + 2 + d, (obj.degree+1) * (i-1) + j) = cutoffFactorial(j-1, d) * obj.knotTimes(i+1).^(j-1-d);
            A((obj.degree+1) * (i-1) + 2 + d, (obj.degree+1) * (i-1) + obj.degree + 1 + j) = -cutoffFactorial(j-1, d) * obj.knotTimes(i+1).^(j-1-d);
        end

    end

end

% For each boundary condition incorporate it into equations
for num_con = 1:nBndCnd
    
    % Order of the derivative upon which condition is placed
    d = obj.boundaryConditions(num_con, 1).DerivativeOrder;
    if (d < 0) || (d > obj.degree)
        error("spline: obj.boundaryConditions the derivative order can't be less than 0 or bigger than polynomial order.");
    end
    
    % Knot is concerned by the condition
    num_knot = obj.boundaryConditions(num_con, 1).KnotOfCondition;
    
    % Polynomial is concerned by the condition
    % By default its the same as the knot
    num_pol = num_knot;
    % Except if it is the last knot, then it is the last polynomial (of
    % which there is one less, so subtract one)
    if num_knot == obj.knotNumber
        num_pol = num_knot - 1;
    end    
    
    % Time at which the condition is placed
    x_c = obj.knotTimes(num_knot);
    
    % For each coefficient dependent upon the order of the derivative
    for j = d+1 : obj.degree+1
        % Equate the d-th order derivative of num_pol-th polynomial as the 
        % num_con-th condition at point x_c with the value given by y_c.
        A((obj.degree+1) * (n-1) + 2 + num_con, (obj.degree+1) * (num_pol-1) + j) = cutoffFactorial(j-1, d) * x_c.^(j-1-d);
    end
    
    % Equate the d-th order derivative of num_pol-th polynomial as the 
    % num_con-th condition at point x_c with the value given by y_c.
    b((obj.degree+1) * (n-1) + 2 + num_con) = 1;
end

end

% Solve the system
invA = pinv(A);
if strcmp(class(invA), "casadi.SX") || strcmp(class(invA), "casadi.MX")
    indb = find(b);
    invA = invA(:, indb);
else
    invA = invA(:, b);
end

% invA = A\logArrayToMat(b);
if nBndCnd > 0
    coefficients = invA(:, 1:end-nBndCnd) * obj.knotValues.' + invA(:, end-nBndCnd+1:end) * [obj.boundaryConditions(:, 1).Value].';
else
    coefficients = invA * obj.knotValues.';
end

end

function res = cutoffFactorial(n, k)
%CUTOFFFACTORIAL of (n, k) where 1 <= k <= n is equal to the product given
%by n * (n - 1) * ... * (n - k + 1).
    res = 1;
    for i = 0 : k - 1
        res = res * (n - i);
    end
end

function mb = logArrayToMat(b)
% find the locations of non-zeros
locind = find(b);
% create a column for each nonzero
mb = zeros(size(b, 1), numel(locind), 'logical');
% for each nonzero set its corresponding location to nz
for nz = 1 : numel(locind)
    mb(locind(nz), nz) = true;
end
end