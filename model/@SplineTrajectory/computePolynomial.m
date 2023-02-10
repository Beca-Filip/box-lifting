function p = computePolynomial(coefficients, computationTime, derivativeOrder)
%COMPUTEPOLYNOMIAL evaluates the polynomial and its derivatives, up to a 
%given order, at a vector of points. 
%
%   p = COMPUTEPOLYNOMIAL(coefficients, computationTime) 
%   Returns a row vector of polynomial values evaluated at times given by 
%   computationTime.
%
%   p = COMPUTEPOLYNOMIAL(coefficients, computationTime, derivativeOrder)
%   Returns a row-major matrix of polynomial and derivative values 
%   evaluated at times given by computationTime. The i-th row corresponds
%   to the evaluations of the (i-1)th derivative.
%
%   Note: Compatible with symbolic and CASADI variables.
%
%   Author: Filip Becanovic
%   Last Modified: 26.01.2023.

    % Input check
    if nargin == 3
    elseif nargin == 2
        derivativeOrder = 0;
    else
        error("Invalid number of inputs.");
    end

    % Coef is a column vector
    coefficients = coefficients(:);

    % Time is row vector
    computationTime = computationTime(:)';

    % Check the order of the polynomial
    polyOrder = length(coefficients) - 1;

    % If order is negative or too high
    if derivativeOrder < 0 && derivativeOrder > polyOrder
        error("Polynomial derivative order must be between zero (0) and the order of the polynomial.")
    end

    % Check the number of time samples
    num_samples = length(computationTime);

    % Prealocate time power matrix
    time = zeros(polyOrder+1, num_samples, class(computationTime));

    % Fill time power matrix
    for i = 1 : polyOrder + 1
        time(i, :) = computationTime.^(i-1);
    end

    % % To use numerically
    % % Prealocate polynomial output
    % p = nan(1 + derivativeOrder, num_samples);
    % 
    % % Calculate the polynomial values
    % p(1, :) = coefficients' * time;

    % To use symbolically
    % Calculate the polynomial values
    p = coefficients' * time;

    % For each additionnal output / derivative
    for d = 1 : derivativeOrder

        % Separate coefs used for this derivative
        deriativeCoefficients = coefficients(d+1:end);

        % For each power of time calculate the multiplicator needed depending
        % on the order of the derivative
        mul = nan(polyOrder - d + 1, 1);
        for i = d : polyOrder
            mul(i - d + 1) = cutoffFactorial(i, d);
        end
        % Multiply coefs with multiplicator
        deriativeCoefficients = deriativeCoefficients .* mul;

    %     % To use numerically
    %     % Calculate output
    %     p(1 + d, :) = deriativeCoefficients' * time(1:polyOrder-d+1, :);

        % To use symbolically
        p = [p; deriativeCoefficients' * time(1:polyOrder-d+1, :)];
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